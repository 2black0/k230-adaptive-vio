/*
    This file is part of EqVIO.

    EqVIO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EqVIO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EqVIO.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "eqvio/Diagnostics.h"
#include "eqvio/LoopTimer.h"
#include "eqvio/VIOFilter.h"
#include "eqvio/VIOFilterSettings.h"
#include "eqvio/VIOSimulator.h"
#include "eqvio/VIOWriter.h"
#include "eqvio/dataserver/dataservers.h"

#include "GIFT/KeyPointFeatureTracker.h"
#include "GIFT/RANSAC.h"
#include "GIFT/PointFeatureTracker.h"
#include "GIFT/Visualisation.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#include "yaml-cpp/yaml.h"

#include <Eigen/SVD>

#include <arpa/inet.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <numeric>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <optional>
#include <unordered_map>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "argparse/argparse.hpp"

#include "eqvio/VIOVisualiser.h"

VisionMeasurement convertGIFTFeatures(const std::vector<GIFT::Feature>& GIFTFeatures, const double& stamp);

/** @brief Resident and peak-resident set size of this process, in kibibytes.
 *
 * `time -v` reports only the peak over the whole run, which cannot say how the
 * footprint is divided between the fixed cost of loading the vision library and
 * the per-frame working set. Reading /proc/self/status at chosen milestones
 * separates the two. Returns {0, 0} where /proc is not mounted.
 */
static std::pair<long, long> residentKiB() {
    std::ifstream status("/proc/self/status");
    std::string key;
    long rss = 0, hwm = 0, value = 0;
    while (status >> key) {
        if (key == "VmRSS:") {
            status >> value;
            rss = value;
        } else if (key == "VmHWM:") {
            status >> value;
            hwm = value;
        }
        status.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return {rss, hwm};
}

/// The output directory with a trailing slash, so log paths can be concatenated.
static std::string outDirOf(std::string dir) {
    if (dir.empty() || dir.back() != '/')
        dir += '/';
    return dir;
}

/** @brief Which components of the adaptive policy are active.
 *
 * The four-arm comparison shows that adaptive gating wins, but adaptive gating
 * is several mechanisms at once -- unconditional vehicle rejection, a pedestrian
 * policy, the starvation floor, scale-adaptive padding, box propagation and
 * duty cycling -- and a single aggregate number cannot say which of them earned
 * the improvement. Each switch here removes exactly one, so the arms differ in
 * one mechanism at a time.
 */
struct AblationFlags {
    bool vehicleReject = true;   ///< off: vehicles are treated like pedestrians.
    bool starvationFloor = true; ///< off: pedestrians are blanket-rejected in every regime.
    bool classDiff = true;       ///< off: one class, every box handled by the pedestrian rule.
    bool usePersonBoxes = true;  ///< off: pedestrian boxes are ignored entirely.
    bool useVehicleBoxes = true; ///< off: vehicle boxes are ignored entirely.
    bool boxPropagation = true;  ///< off: boxes are frozen between detections instead of moved.
};

static AblationFlags ablation;

/** @brief Per-frame dynamic-object masks, looked up by timestamp.
 *
 * GIFT's mask is a *keep* mask, following OpenCV: non-zero means the pixel may
 * be used, both in goodFeaturesToTrack and when culling tracks that drift. The
 * masks on disk are the opposite -- 255 marks the dynamic object -- so they are
 * inverted on load. Getting this backwards tracks only the moving objects and
 * still runs, which is why it is done in one place.
 *
 * Frames are looked up by nearest timestamp rather than exact match: the image
 * stamp has cameraLag subtracted before it reaches here.
 */
class DynamicMaskSource {
  public:
    DynamicMaskSource(const std::string& dir, int dilatePx) : dilate(dilatePx) {
        if (dir.empty() || !std::filesystem::exists(dir))
            return;
        for (const auto& e : std::filesystem::directory_iterator(dir)) {
            if (e.path().extension() != ".png")
                continue;
            try {
                stamps.emplace_back(std::stoll(e.path().stem().string()), e.path().string());
            } catch (const std::exception&) {
            }
        }
        std::sort(stamps.begin(), stamps.end());
        if (dilate > 0)
            kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilate + 1, 2 * dilate + 1));
    }

    bool active() const { return !stamps.empty(); }
    size_t size() const { return stamps.size(); }
    long long applied() const { return nApplied; }
    long long missed() const { return nMissed; }

    /// Keep-mask for this stamp, or an empty Mat if there is no frame near it.
    cv::Mat keepMaskAt(double stampSeconds) {
        if (stamps.empty())
            return cv::Mat();
        const long long ns = static_cast<long long>(std::llround(stampSeconds * 1e9));
        auto it = std::lower_bound(stamps.begin(), stamps.end(), std::make_pair(ns, std::string()));
        long long best = -1;
        std::string bestPath;
        for (auto c : {it, (it == stamps.begin() ? it : std::prev(it))}) {
            if (c == stamps.end())
                continue;
            const long long d = std::llabs(c->first - ns);
            if (best < 0 || d < best) {
                best = d;
                bestPath = c->second;
            }
        }
        if (best < 0 || best > 30000000LL) { // 30 ms: no frame corresponds
            ++nMissed;
            return cv::Mat();
        }
        cv::Mat dyn = cv::imread(bestPath, cv::IMREAD_GRAYSCALE);
        if (dyn.empty()) {
            ++nMissed;
            return cv::Mat();
        }
        if (dilate > 0)
            cv::dilate(dyn, dyn, kernel);
        cv::Mat keep;
        cv::bitwise_not(dyn, keep);
        ++nApplied;
        return keep;
    }

  private:
    std::vector<std::pair<long long, std::string>> stamps;
    int dilate = 0;
    cv::Mat kernel;
    long long nApplied = 0, nMissed = 0;
};


/** @brief Reject only the features a moving object actually makes wrong.
 *
 * Blanket masking deletes every feature on a moving object. Measured against
 * ground-truth ego-motion, 19-92 % of those features are epipolar-consistent
 * depending on the scene -- a distant or slowly moving vehicle produces
 * observations a static-world estimator uses perfectly well. In city_night,
 * where 71-81 % are consistent, blanket masking reliably makes the trajectory
 * worse (docs/32 section 5).
 *
 * The share of features that are dynamic *and* inconsistent predicts the
 * benefit of masking with Spearman r = -0.72 (p = 0.03), where the share of
 * dynamic features alone does not (p = 0.19). So the mask should mark
 * suspicion, not guilt, and geometry should decide.
 *
 * The model is fitted only on features OUTSIDE the mask, which is the one set
 * known to be static, and features outside the mask are never rejected -- a
 * geometric gate applied to them was measured to be worse than doing nothing
 * (docs/33). Only masked features face the test.
 */

/** @brief Detector boxes carried across frames by the tracker's own optical flow.
 *
 * yolov8n_320 costs 57.6 ms on the KPU against EqVIO's 28.4 ms, so detecting
 * every frame caps the pipeline near 11 fps. Detecting every Nth frame and
 * *freezing* the mask between is not safe -- on advio-20 a 2-frame freeze
 * diverged the filter to 60 km, because a stale box sits over background while
 * its object has moved. Boxes have to move with the scene.
 *
 * The displacement comes from features GIFT has already tracked this frame, so
 * propagation adds no optical flow of its own: for each box, take the median
 * pixel displacement of the features whose previous position fell inside it.
 * That absorbs camera motion and object motion together without separating them
 * or knowing depth. Boxes with too few features inside fall back to the median
 * over all features, which is dominated by ego-motion -- still far better than
 * leaving the box where it was.
 *
 * Boxes grow slightly per propagated frame, because a rigid rectangle cannot
 * follow an object that rotates or approaches, and are dropped once they leave
 * the image. Growth is cheap under selective rejection, where a wrongly covered
 * static feature is kept anyway once it passes the geometric test (docs/33).
 */
class PropagatedMaskSource {
  public:
    /// How a box is carried from one frame to the next between detections.
    enum class Track {
        Features,      ///< reuse the displacements GIFT already computed. 0.04 ms/frame.
        Roi,           ///< seed points once per detection, one KLT for all boxes. 4.6 ms/frame.
        Redetect,      ///< fresh goodFeaturesToTrack inside every box every frame. 8.4 ms/frame.
        Global,        ///< global ego-motion median only; never looks inside the box (ablation B4).
        InternalOnly,  ///< internal-feature median only; no global fallback, box waits (ablation B5).
    };

    struct LabeledBox {
        cv::Rect2f rect;
        bool isVehicle = false;
        cv::Point2f disp{0.f, 0.f}; ///< last per-frame displacement, for motion-proportional FADM dilation.
    };

    /** @brief Freshness-Aware Dynamic Masking parameters (plan Section 3.3).
     *
     * A stale detection cannot be treated with static box boundaries: the
     * object has kept moving while the packet sat in the accelerator queue,
     * and propagated coordinates only follow it to within optical-flow error.
     * FADM dilates each presented box by
     *
     *   pad = pad_base + pad_age * age_ms + pad_motion * |displacement|,
     *
     * computed fresh every frame from the RAW propagated rectangle, so the
     * padding never accumulates and collapses back to the base the moment a
     * fresh detection arrives. A negative padBase disables FADM entirely and
     * preserves the legacy behaviour of presenting raw boxes.
     *
     * The dilation is applied where the boxes are handed to the gate, not
     * where they are propagated: with FADM active the arm's own gate padding
     * (--dynmaskVehPad/--dynmaskPersonPad) should be zero, so the effective
     * boundary is exactly the modelled uncertainty.
     */
    struct FadmParams {
        double padBase = -1.0;  ///< Fixed base padding in px; negative disables FADM.
        double padAge = 0.0;    ///< Dilation per ms of detection age (alpha_age).
        double padMotion = 0.0; ///< Dilation per px/frame of box displacement (beta_motion).
        double padMax = 60.0;   ///< Ceiling on total dilation; severe staleness must not eat the frame.
    };

    /** @brief Discrete-event emulation of the asynchronous accelerator channel.
     *
     * A duty-cycled detector (RTX GPU on the host, KPU on the board) does not
     * return boxes for frame t_k until t_k + T_lat, with T_lat jittered by the
     * queue and occasionally lost outright. Replaying the per-frame detection
     * CSVs through this model turns any recorded run into the same run the
     * asynchronous hardware would have produced, without OS scheduling noise:
     * the emulator decides only WHEN a detection becomes visible, never what
     * it contains.
     *
     * delayMs/jitterMs/dropRate are seeded from rngSeed, so a scenario is
     * reproducible and a seed sweep measures the method, not the draw.
     */
    struct TimingEmulation {
        double delayMs = 0.0;   ///< T_lat: accelerator inference + transfer latency.
        double jitterMs = 0.0;  ///< +/- half-width of the uniform latency jitter.
        double dropRate = 0.0;  ///< Probability that a detection packet never arrives.
        int maxAgeFrames = 0;   ///< TTL horizon: boxes older than this (from the detection's SOURCE frame) expire. 0 disables.
        int rngSeed = 0;        ///< Seed for the jitter and drop draws.
        bool blocking = false;  ///< Synchronous semantics (ablation B10): the estimator thread STALLS for T_lat on every detection frame instead of consuming the result asynchronously. Boxes are then current, but the frame deadline is breached by construction.
    };

    /// Freshness of the active detection, for the per-frame telemetry log.
    enum class DetStatus { None, Fresh, Propagated, Stale, Expired, Dropped };

    static const char* detStatusName(DetStatus s) {
        switch (s) {
        case DetStatus::Fresh: return "fresh";
        case DetStatus::Propagated: return "propagated";
        case DetStatus::Stale: return "stale";
        case DetStatus::Expired: return "expired";
        case DetStatus::Dropped: return "dropped";
        default: return "none";
        }
    }

    PropagatedMaskSource(const std::string& csv, int interval, double grow, double conf, Track track,
        double vehConf, const TimingEmulation& timing, const FadmParams& fadm,
        const std::string& liveEndpoint = "")
        : interval(std::max(1, interval)), grow(grow), track(track), timing(timing), rng(timing.rngSeed),
          jitterDist(-timing.jitterMs, timing.jitterMs), dropDist(timing.dropRate), fadm(fadm) {
        if (!liveEndpoint.empty()) {
            openLiveSocket(liveEndpoint);
            return;
        }
        if (csv.empty() || !std::filesystem::exists(csv))
            return;
        std::ifstream f(csv);
        std::string line;
        std::getline(f, line); // header
        static const std::set<std::string> vehicles = {"car", "bus", "truck", "motorcycle"};
        static const std::set<std::string> movable = {"person", "bicycle", "car", "motorcycle", "bus",
            "train", "truck", "boat", "dog", "cat", "horse", "bird"};
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            std::string path, cid, cname, sconf, sx, sy, sw, sh;
            if (!std::getline(ss, path, ',') || !std::getline(ss, cid, ',') ||
                !std::getline(ss, cname, ',') || !std::getline(ss, sconf, ',') ||
                !std::getline(ss, sx, ',') || !std::getline(ss, sy, ',') ||
                !std::getline(ss, sw, ',') || !std::getline(ss, sh, ','))
                continue;
            if (!movable.count(cname))
                continue;
            try {
                const double sc = std::stod(sconf);
                const bool isVeh = (vehicles.count(cname) > 0);
                if (isVeh ? (sc < vehConf) : (sc < conf))
                    continue;
                const std::string stem = std::filesystem::path(path).stem().string();
                boxesByStamp[std::stoll(stem)].push_back(
                    {cv::Rect2f(std::stof(sx), std::stof(sy), std::stof(sw), std::stof(sh)), isVeh});
                ++nBoxes;
            } catch (const std::exception&) {
            }
        }
        for (const auto& kv : boxesByStamp)
            stamps.push_back(kv.first);
        std::sort(stamps.begin(), stamps.end());
    }

    ~PropagatedMaskSource() {
        if (liveSocket >= 0)
            ::close(liveSocket);
    }

    bool active() const { return liveSocket >= 0 || !stamps.empty(); }
    size_t detections() const { return nBoxes; }
    /// Boxes as the gate should see them: raw rectangles dilated by the FADM
    /// padding (equal to the raw rectangles when FADM is disabled).
    std::vector<cv::Rect2f> currentBoxes() const {
        std::vector<cv::Rect2f> res;
        res.reserve(paddedBoxes.size());
        for (const auto& b : paddedBoxes) res.push_back(b.rect);
        return res;
    }
    std::vector<cv::Rect2f> currentVehicleBoxes() const {
        std::vector<cv::Rect2f> res;
        for (const auto& b : paddedBoxes) if (b.isVehicle) res.push_back(b.rect);
        return res;
    }
    std::vector<cv::Rect2f> currentPersonBoxes() const {
        std::vector<cv::Rect2f> res;
        for (const auto& b : paddedBoxes) if (!b.isVehicle) res.push_back(b.rect);
        return res;
    }

    /** Advance the boxes without rasterising a mask.
     *
     * Selective rejection only asks whether a feature lies inside a box, which
     * is a handful of rectangle tests. Building a 225x400 CV_8U mask every frame
     * to answer that cost 6.9 ms/frame on the board -- more than three times the
     * geometric test it feeds. The mask image is only needed by the blanket path,
     * where GIFT consumes it directly.
     */
    void advance(double stampSeconds, const std::vector<GIFT::Feature>& features, const cv::Mat& image) {
        if (liveSocket < 0 && stamps.empty())
            return;
        curImage = image;
        imageW = static_cast<float>(image.cols);
        imageH = static_cast<float>(image.rows);
        stepFrame(stampSeconds, features);
        rememberPixels(features);
        prevImage = curImage;
    }
    long long detected() const { return nDetected; }
    long long propagated() const { return nPropagated; }
    long long frozen() const { return nFrozen; }
    long long packetsDropped() const { return nPacketsDropped; }
    long long ttlExpiries() const { return nExpired; }
    long long packetsArrived() const { return nArrived; }

    /// Freshness telemetry of the active detection, filled by the last advance.
    DetStatus status() const { return lastStatus; }
    const char* statusName() const { return detStatusName(lastStatus); }
    /// Age of the active boxes counted from their SOURCE frame; -1 when none.
    int ageFrames() const { return lastAgeFrames; }
    /// Age in ms from the source frame's acquisition stamp; -1 when none.
    double ageMs() const { return lastAgeMs; }
    /// Queue latency of the last adopted packet (arrival stamp - source stamp), ms.
    double latencyMs() const { return lastLatencyMs; }
    /// Boxes moved by internal / global / frozen displacement on the last propagation.
    int srcInternal() const { return nSrcInternal; }
    int srcGlobal() const { return nSrcGlobal; }
    int srcFrozen() const { return nSrcFrozen; }
    /// Mean FADM padding over the presented boxes; -1 when none or FADM off.
    double padMean() const { return lastPadMean; }

    /// Advance the boxes to this frame, then rasterise them into a keep mask.
    cv::Mat keepMaskFor(double stampSeconds, const std::vector<GIFT::Feature>& features, cv::Size size,
        const cv::Mat& image = cv::Mat()) {
        if (liveSocket < 0 && stamps.empty())
            return cv::Mat();
        curImage = image;
        stepFrame(stampSeconds, features);
        rememberPixels(features);
        prevImage = curImage;

        cv::Mat keep(size, CV_8U, cv::Scalar(255));
        for (const auto& b : paddedBoxes) {
            cv::Rect r(cvRound(b.rect.x), cvRound(b.rect.y), cvRound(b.rect.width), cvRound(b.rect.height));
            r &= cv::Rect(0, 0, size.width, size.height);
            if (r.width > 0 && r.height > 0)
                keep(r).setTo(0);
        }
        return keep;
    }

  private:
    void openLiveSocket(const std::string& endpoint) {
        const std::string prefix = "udp://127.0.0.1:";
        if (endpoint.rfind(prefix, 0) != 0)
            throw std::runtime_error("--dyndetLive must use udp://127.0.0.1:<port>");
        const int port = std::stoi(endpoint.substr(prefix.size()));
        if (port < 1 || port > 65535)
            throw std::runtime_error("--dyndetLive port must be in 1..65535");

        liveSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (liveSocket < 0)
            throw std::runtime_error("cannot create live detector UDP socket: " + std::string(std::strerror(errno)));
        int flags = ::fcntl(liveSocket, F_GETFL, 0);
        if (flags < 0 || ::fcntl(liveSocket, F_SETFL, flags | O_NONBLOCK) < 0) {
            ::close(liveSocket);
            liveSocket = -1;
            throw std::runtime_error("cannot make live detector UDP socket non-blocking");
        }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(liveSocket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
            ::close(liveSocket);
            liveSocket = -1;
            throw std::runtime_error("cannot bind live detector UDP endpoint " + endpoint);
        }
    }

    void pollLivePackets(double estimatorStamp) {
        if (liveSocket < 0)
            return;
        std::array<char, 65536> buffer{};
        while (true) {
            const ssize_t received = ::recv(liveSocket, buffer.data(), buffer.size() - 1, MSG_DONTWAIT);
            if (received <= 0)
                break;
            buffer[static_cast<size_t>(received)] = '\0';
            const std::string packet(buffer.data(), static_cast<size_t>(received));
            const size_t sep = packet.find('|');
            if (sep == std::string::npos)
                continue;
            try {
                PendingPacket p;
                p.sourceStamp = std::stoll(packet.substr(0, sep)) * 1e-9;
                p.arrivalStamp = estimatorStamp;
                const double periodMs = framePeriodMs > 0.0 ? framePeriodMs : 50.0;
                const long long ageFrames = std::max<long long>(0, std::llround(
                    (estimatorStamp - p.sourceStamp) * 1e3 / periodMs));
                p.detFrameIdx = ageFrames >= static_cast<long long>(frameIndex)
                    ? 0 : frameIndex - static_cast<size_t>(ageFrames);
                const std::string payload = packet.substr(sep + 1);
                if (!payload.empty()) {
                    std::stringstream boxesStream(payload);
                    std::string item;
                    while (std::getline(boxesStream, item, ';')) {
                        if (item.empty())
                            continue;
                        std::stringstream fields(item);
                        std::string sx, sy, sw, sh, sv;
                        if (!std::getline(fields, sx, ',') || !std::getline(fields, sy, ',') ||
                            !std::getline(fields, sw, ',') || !std::getline(fields, sh, ',') ||
                            !std::getline(fields, sv, ','))
                            continue;
                        p.boxes.push_back({cv::Rect2f(std::stof(sx), std::stof(sy), std::stof(sw), std::stof(sh)),
                            std::stoi(sv) != 0});
                    }
                }
                pending.push_back(std::move(p));
                ++nDetected;
            } catch (const std::exception&) {
                // A malformed datagram is dropped; the estimator remains live.
                ++nPacketsDropped;
            }
        }
    }

    /** One estimator frame of the discrete-event box pipeline.
     *
     * In CSV mode the detector thread is modelled, not executed: every
     * `interval` frames an inference is *started* on that frame's image, and
     * its boxes become visible only on the first estimator frame at or after
     * source + T_lat (+ jitter). In live mode the same pending-packet policy
     * consumes datagrams from the external worker. A dropped packet never
     * becomes visible. Until a packet arrives the previous boxes are carried by
     * optical flow (or frozen, under the ablation), and a TTL horizon bounds
     * how long a lost detection keeps masking the wrong pixels.
     */
    void stepFrame(double stampSeconds, const std::vector<GIFT::Feature>& features) {
        if (lastFrameStamp > 0.0) {
            const double dtMs = (stampSeconds - lastFrameStamp) * 1e3;
            if (dtMs > 1.0 && dtMs < 200.0)
                framePeriodMs = framePeriodMs < 0.0 ? dtMs : 0.7 * framePeriodMs + 0.3 * dtMs;
        }
        lastFrameStamp = stampSeconds;
        lastStatus = DetStatus::None;
        nSrcInternal = nSrcGlobal = nSrcFrozen = 0;
        bool adopted = false;

        // In live mode the accelerator is a separate process. Polling is
        // non-blocking: late or missing datagrams are handled by the same
        // propagation/TTL policy as the emulated channel.
        if (liveSocket >= 0) {
            pollLivePackets(stampSeconds);
        // The accelerator picks up this frame's image for inference.
        } else if (frameIndex % interval == 0) {
            loopTimer.startTiming("nn_detect");
            const std::vector<LabeledBox> det = nearestBoxes(stampSeconds);
            loopTimer.endTiming("nn_detect");
            ++nDetected;
            const double latency = std::max(0.0, timing.delayMs + (timing.jitterMs > 0.0 ? jitterDist(rng) : 0.0));
            if (timing.dropRate > 0.0 && dropDist(rng)) {
                ++nPacketsDropped;
                lastStatus = DetStatus::Dropped;
            } else if (timing.blocking) {
                // Ablation B10: the estimator thread blocks on the detector.
                // The boxes describe THIS frame (age 0), but the stall lands
                // inside the frame budget -- the scheduling contrast the
                // queue ablation measures.
                std::this_thread::sleep_for(
                    std::chrono::microseconds(static_cast<long long>(latency * 1e3)));
                boxes = det;
                detFrameIndex = frameIndex;
                sourceStamp = stampSeconds;
                lastLatencyMs = latency;
                seedBoxPoints();
                ++nArrived;
                adopted = true;
                lastStatus = DetStatus::Fresh;
            } else {
                PendingPacket p;
                p.boxes = det;
                p.sourceStamp = stampSeconds;
                p.detFrameIdx = frameIndex;
                p.arrivalStamp = stampSeconds + latency * 1e-3;
                pending.push_back(std::move(p));
            }
        }

        // Adopt every packet that has arrived by this frame; the newest wins,
        // as it would over a real queue fed by a single worker.
        while (!pending.empty() && pending.front().arrivalStamp <= stampSeconds) {
            boxes = pending.front().boxes;
            detFrameIndex = pending.front().detFrameIdx;
            sourceStamp = pending.front().sourceStamp;
            lastLatencyMs = (stampSeconds - sourceStamp) * 1e3;
            seedBoxPoints();
            adopted = true;
            ++nArrived;
            pending.pop_front();
        }
        if (adopted)
            lastStatus = DetStatus::Fresh;

        // TTL expiry: a lost detection must not mask wrong pixels forever.
        if (timing.maxAgeFrames > 0 && !boxes.empty() &&
            static_cast<long long>(frameIndex - detFrameIndex) > timing.maxAgeFrames) {
            boxes.clear();
            boxPoints.clear();
            ++nExpired;
            lastStatus = DetStatus::Expired;
        }

        if (lastStatus == DetStatus::None && !boxes.empty()) {
            if (ablation.boxPropagation) {
                propagate(features);
                ++nPropagated;
                // Stale = older than a healthy channel can explain. The nominal
                // worst age is one detector period plus the accelerator transit
                // (delay + worst jitter, in frames); beyond that the boxes have
                // survived a missed refresh -- a dropped or badly queued packet.
                const double periodMs = framePeriodMs > 0.0 ? framePeriodMs : 50.0;
                const int transitFrames = timing.delayMs > 0.0
                    ? static_cast<int>(std::ceil((timing.delayMs + timing.jitterMs) / periodMs))
                    : 0;
                lastStatus = static_cast<long long>(frameIndex - detFrameIndex) > interval + transitFrames
                    ? DetStatus::Stale
                    : DetStatus::Propagated;
            } else {
                // Ablation: boxes frozen where the last detection left them. This is
                // the arm that shows what propagation buys, and it is also what a
                // naive duty cycle does.
                ++nFrozen;
                nSrcFrozen = static_cast<int>(boxes.size());
            }
        }

        lastAgeFrames = boxes.empty() ? -1 : static_cast<int>(frameIndex - detFrameIndex);
        lastAgeMs = (boxes.empty() || sourceStamp <= 0.0) ? -1.0 : (stampSeconds - sourceStamp) * 1e3;

        // FADM presentation pass: dilate every raw box by its freshness-aware
        // padding. Recomputed from scratch each frame, so the padding tracks
        // the current age and displacement instead of accumulating.
        paddedBoxes.clear();
        double padSum = 0.0;
        for (const auto& b : boxes) {
            const double p = fadmPadFor(b);
            padSum += p;
            paddedBoxes.push_back(
                {cv::Rect2f(b.rect.x - p, b.rect.y - p, b.rect.width + 2.0 * p, b.rect.height + 2.0 * p),
                    b.isVehicle, b.disp});
        }
        lastPadMean = (boxes.empty() || fadm.padBase < 0.0) ? -1.0 : padSum / static_cast<double>(boxes.size());
        ++frameIndex;
    }

  private:
    /// FADM dilation for one box: base + age term + motion term, clamped.
    double fadmPadFor(const LabeledBox& b) const {
        if (fadm.padBase < 0.0)
            return 0.0;
        const double age = lastAgeMs > 0.0 ? lastAgeMs : 0.0;
        const double motion = std::hypot(b.disp.x, b.disp.y);
        return std::min(fadm.padMax, fadm.padBase + fadm.padAge * age + fadm.padMotion * motion);
    }

    std::vector<LabeledBox> nearestBoxes(double stampSeconds) const {
        const long long ns = static_cast<long long>(std::llround(stampSeconds * 1e9));
        auto it = std::lower_bound(stamps.begin(), stamps.end(), ns);
        long long best = -1, bestStamp = 0;
        for (auto c : {it, (it == stamps.begin() ? it : std::prev(it))}) {
            if (c == stamps.end())
                continue;
            const long long d = std::llabs(*c - ns);
            if (best < 0 || d < best) {
                best = d;
                bestStamp = *c;
            }
        }
        if (best < 0 || best > 30000000LL)
            return {};
        const auto it2 = boxesByStamp.find(bestStamp);
        if (it2 == boxesByStamp.end()) {
            std::cerr << "nearestBoxes: no entry for " << bestStamp << " (ns=" << ns << ")" << std::endl;
            return {};
        }
        return it2->second;
    }

    /** Track one box with points detected inside it, when the tracker has none there.
     *
     * With 15-25 features spread over the image and masks covering 2.6 % of it
     * (advio-20), most boxes contain no tracked feature at all, so the median
     * below falls back to ego-motion and the box stops following its object.
     * Detecting a few points inside the box recovers object motion; it costs one
     * small-ROI KLT, and only for boxes that need it on propagated frames.
     */
    bool trackBoxDirect(const cv::Mat& prev, const cv::Mat& cur, const cv::Rect2f& b, cv::Point2f& disp) const {
        if (prev.empty() || cur.empty())
            return false;
        cv::Rect r(cvRound(b.x), cvRound(b.y), cvRound(b.width), cvRound(b.height));
        r &= cv::Rect(0, 0, prev.cols, prev.rows);
        if (r.width < 6 || r.height < 6)
            return false;
        std::vector<cv::Point2f> pts;
        cv::goodFeaturesToTrack(prev(r), pts, 12, 0.01, 3);
        if (pts.size() < 3)
            return false;
        for (auto& p : pts)
            p += cv::Point2f(static_cast<float>(r.x), static_cast<float>(r.y));
        std::vector<cv::Point2f> nxt;
        std::vector<uchar> st;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev, cur, pts, nxt, st, err, cv::Size(15, 15), 2);
        std::vector<cv::Point2f> d;
        for (size_t i = 0; i < pts.size(); ++i)
            if (st[i])
                d.push_back(nxt[i] - pts[i]);
        if (d.size() < 3)
            return false;
        disp = median(d);
        return true;
    }

    void propagate(const std::vector<GIFT::Feature>& features) {
        std::vector<cv::Point2f> allDisp;
        std::vector<std::pair<cv::Point2f, cv::Point2f>> moved; // previous, displacement
        for (const auto& f : features) {
            auto it = prevPixels.find(f.idNumber);
            if (it == prevPixels.end())
                continue;
            const cv::Point2f d = f.camCoordinates - it->second;
            moved.emplace_back(it->second, d);
            allDisp.push_back(d);
        }
        const cv::Point2f global = median(allDisp);

        // One KLT call for every box's points at once. Calling it per box rebuilt
        // the full-image pyramid each time, which measured 8.4 ms median and
        // 29.9 ms p95 on the board -- more than the tracker it was helping.
        // Points are seeded once per detection and tracked forward from then on,
        // so goodFeaturesToTrack runs every Nth frame rather than every frame,
        // and each box follows a continuous track instead of a fresh detection.
        std::vector<cv::Point2f> flat, flatNext;
        std::vector<size_t> owner;
        if (track == Track::Roi && !prevImage.empty() && !curImage.empty()) {
            for (size_t i = 0; i < boxes.size(); ++i)
                for (const auto& pt : boxPoints[i]) {
                    flat.push_back(pt);
                    owner.push_back(i);
                }
            if (!flat.empty()) {
                std::vector<uchar> st;
                std::vector<float> err;
                cv::calcOpticalFlowPyrLK(prevImage, curImage, flat, flatNext, st, err, cv::Size(15, 15), 2);
                std::vector<std::vector<cv::Point2f>> keptPts(boxes.size());
                std::vector<std::vector<cv::Point2f>> perBox(boxes.size());
                for (size_t k = 0; k < flat.size(); ++k) {
                    if (!st[k])
                        continue;
                    perBox[owner[k]].push_back(flatNext[k] - flat[k]);
                    keptPts[owner[k]].push_back(flatNext[k]);
                }
                boxDisp.assign(boxes.size(), cv::Point2f(0.f, 0.f));
                boxHasDisp.assign(boxes.size(), false);
                for (size_t i = 0; i < boxes.size(); ++i)
                    if (perBox[i].size() >= 3) {
                        boxDisp[i] = median(perBox[i]);
                        boxHasDisp[i] = true;
                    }
                boxPoints.swap(keptPts);
            }
        }

        std::vector<LabeledBox> next;
        std::vector<std::vector<cv::Point2f>> nextPts;
        for (size_t i = 0; i < boxes.size(); ++i) {
            const cv::Rect2f& b = boxes[i].rect;
            cv::Point2f d;
            if (track == Track::Roi && i < boxHasDisp.size() && boxHasDisp[i]) {
                d = boxDisp[i];
                ++nSrcInternal; // tracked from points seeded inside this box
            } else if (track == Track::Redetect && trackBoxDirect(prevImage, curImage, b, d)) {
                // fresh points inside this box, every frame: most accurate, dearest
                ++nSrcInternal;
            } else if (track == Track::Global) {
                // Ablation B4: ego-motion only, the box interior is ignored.
                d = global;
                ++nSrcGlobal;
            } else if (track == Track::InternalOnly) {
                // Ablation B5: no fallback -- a box without internal features
                // waits where it is rather than borrowing ego-motion.
                std::vector<cv::Point2f> inside;
                for (const auto& m : moved)
                    if (b.contains(m.first))
                        inside.push_back(m.second);
                if (inside.size() >= 3) {
                    d = median(inside);
                    ++nSrcInternal;
                } else {
                    d = cv::Point2f(0.f, 0.f);
                    ++nSrcFrozen;
                }
            } else {
                std::vector<cv::Point2f> inside;
                for (const auto& m : moved)
                    if (b.contains(m.first))
                        inside.push_back(m.second);
                if (inside.size() >= 3) {
                    d = median(inside);
                    ++nSrcInternal;
                } else {
                    d = global;
                    ++nSrcGlobal;
                }
            }
            cv::Rect2f nb(b.x + d.x - grow, b.y + d.y - grow, b.width + 2 * grow, b.height + 2 * grow);
            if (nb.x + nb.width > 0 && nb.y + nb.height > 0 && nb.x < imageW && nb.y < imageH) {
                next.push_back({nb, boxes[i].isVehicle, d});
                if (track == Track::Roi)
                    nextPts.push_back(i < boxPoints.size() ? boxPoints[i] : std::vector<cv::Point2f>());
            }
        }
        boxes.swap(next);
        if (track == Track::Roi)
            boxPoints.swap(nextPts);
    }

    /// Seed tracking points inside each freshly detected box, once per detection.
    void seedBoxPoints() {
        boxPoints.assign(boxes.size(), {});
        if (track != Track::Roi || curImage.empty())
            return;
        for (size_t i = 0; i < boxes.size(); ++i) {
            cv::Rect r(cvRound(boxes[i].rect.x), cvRound(boxes[i].rect.y), cvRound(boxes[i].rect.width), cvRound(boxes[i].rect.height));
            r &= cv::Rect(0, 0, curImage.cols, curImage.rows);
            if (r.width < 6 || r.height < 6)
                continue;
            std::vector<cv::Point2f> pts;
            cv::goodFeaturesToTrack(curImage(r), pts, 12, 0.01, 3);
            for (auto& pt : pts)
                pt += cv::Point2f(static_cast<float>(r.x), static_cast<float>(r.y));
            boxPoints[i] = std::move(pts);
        }
    }

    static cv::Point2f median(std::vector<cv::Point2f> v) {
        if (v.empty())
            return {0.f, 0.f};
        const size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + m, v.end(),
            [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });
        const float mx = v[m].x;
        std::nth_element(v.begin(), v.begin() + m, v.end(),
            [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        return {mx, v[m].y};
    }

    void rememberPixels(const std::vector<GIFT::Feature>& features) {
        prevPixels.clear();
        for (const auto& f : features)
            prevPixels[f.idNumber] = f.camCoordinates;
    }

  public:
    float imageW = 1e9f, imageH = 1e9f;

  private:
    std::map<long long, std::vector<LabeledBox>> boxesByStamp;
    std::vector<long long> stamps;
    std::vector<LabeledBox> boxes;
    std::vector<std::vector<cv::Point2f>> boxPoints;
    std::vector<cv::Point2f> boxDisp;
    std::vector<bool> boxHasDisp;
    std::map<int, cv::Point2f> prevPixels;
    cv::Mat prevImage, curImage;
    int interval;
    double grow;
    Track track = Track::Features;

    /** A detection in flight from the (emulated) accelerator to the estimator. */
    struct PendingPacket {
        std::vector<LabeledBox> boxes;
        double sourceStamp = 0.0;   ///< Acquisition stamp of the image the detector consumed.
        double arrivalStamp = 0.0;  ///< First estimator time at which the boxes are visible.
        size_t detFrameIdx = 0;     ///< frameIndex of the source frame, for age-in-frames.
    };
    std::deque<PendingPacket> pending;
    TimingEmulation timing;
    std::mt19937 rng;
    std::uniform_real_distribution<double> jitterDist;
    std::bernoulli_distribution dropDist;
    FadmParams fadm;
    std::vector<LabeledBox> paddedBoxes; ///< FADM-dilated presentation copies, rebuilt every frame.
    int liveSocket = -1; ///< Non-blocking loopback transport for a live detector worker.

    size_t frameIndex = 0, nBoxes = 0;
    size_t detFrameIndex = 0;      ///< frameIndex of the frame the active boxes were detected on.
    double sourceStamp = -1.0;     ///< Acquisition stamp of the active boxes' source frame.
    double lastFrameStamp = -1.0;  ///< Previous estimator stamp, to estimate the frame period.
    double framePeriodMs = -1.0;   ///< Smoothed camera period (ADVIO: ~50 ms).
    DetStatus lastStatus = DetStatus::None;
    int lastAgeFrames = -1;
    double lastAgeMs = -1.0;
    double lastLatencyMs = 0.0;
    double lastPadMean = -1.0;
    int nSrcInternal = 0, nSrcGlobal = 0, nSrcFrozen = 0;
    long long nDetected = 0, nPropagated = 0, nFrozen = 0, nPacketsDropped = 0, nExpired = 0, nArrived = 0;
};

/** @brief What the gate did on one frame, for the mechanism-validation log.
 *
 * The paper's claims about the policy are counting claims -- how often the
 * starvation floor fires, how many pedestrian features it readmits, and whether
 * those readmissions were geometrically tested -- so the counts have to come
 * out of the code that makes the decision rather than be inferred afterwards
 * from a trajectory.
 *
 * The distinction that matters most is `readmittedUnverified`. Residual
 * verification needs eight background correspondences to fit an essential
 * matrix, and the starvation branch only runs when fewer than N_safe (deployed:
 * four) are left, so at the deployed floor the verification branch is
 * unreachable and readmission is untested. That is a limitation of the method,
 * and it is only visible if the two are counted separately.
 */
struct GateStats {
    int nTracked = 0;      ///< Features the tracker produced this frame.
    int nVehBoxes = 0;     ///< Vehicle boxes active this frame.
    int nPersonBoxes = 0;  ///< Pedestrian boxes active this frame.
    int nVehRejected = 0;  ///< Features dropped by the unconditional vehicle rule.
    int nClean = 0;        ///< Background features: outside every box. This is |C_k|.
    int nSuspect = 0;      ///< Features inside a pedestrian box, facing the policy.
    int nPedRejected = 0;  ///< Pedestrian features dropped in the nominal regime.
    int nBoxRejected = 0;  ///< Features dropped by the undifferentiated blanket rule.
    int nTested = 0;       ///< Suspect features that actually met a residual test.
    int nModelFailures = 0; ///< Suspect features kept without a valid geometric model.
    int nReadmittedVerified = 0;   ///< Readmitted after passing that test.
    int nReadmittedUnverified = 0; ///< Readmitted without any test -- the untested rescue.
    int nOut = 0;          ///< Features handed to the filter.
    int nRiskRemoved = 0;  ///< Features rejected by the risk-information policy.
    int nRiskWeighted = 0; ///< Suspect features retained with covariance inflation.
    int nTemporalMoving = 0; ///< Features whose temporal state is moving.
    double riskMean = 0.0; ///< Mean risk over features considered by the adaptive policy.
    double informationMean = 0.0; ///< Mean information proxy over considered features.
    double weightMean = 1.0; ///< Mean assigned measurement weight for retained features.
    /// nominal | starvation | fallback | passthrough. See gateRegimeName.
    int regime = 0;

    enum Regime { Passthrough = 0, Nominal = 1, Starvation = 2, Fallback = 3 };

    void reset() { *this = GateStats(); }
};

static GateStats gateStats;

enum class AdaptiveFeatureState { Unknown, Static, SemiStatic, Moving };

struct AdaptiveFeatureTrack {
    double risk = 0.0;
    int highEvidence = 0;
    int lowEvidence = 0;
    size_t lastFrame = 0;
    AdaptiveFeatureState state = AdaptiveFeatureState::Unknown;
};

/** Temporal memory for the risk-aware adaptive policy.
 *
 * Feature ids are stable for the lifetime of a GIFT track. The map is bounded
 * by retiring ids that have not been observed for a short horizon, which also
 * prevents an id recycle from inheriting a stale dynamic decision.
 */
static std::unordered_map<int, AdaptiveFeatureTrack> adaptiveFeatureTracks;
static size_t adaptiveFrame = 0;

/** @brief Inertial motion signals for the adaptive gate.
 *
 * Two cheap products of the IMU stream that the gate otherwise never sees:
 *
 * 1. Motion energy -- sliding-window standard deviations of the gyro and
 *    accelerometer magnitudes. These separate "the platform is being shaken
 *    or spun" from "the scene is changing on its own", which a purely visual
 *    gate cannot do: a feature-count collapse while the IMU is quiet means
 *    dynamic contamination, while the same collapse under violent rotation
 *    means ordinary tracking failure.
 *
 * 2. The gyro-integrated rotation since the last image. With the inter-frame
 *    rotation known, the static-world residual no longer needs an essential
 *    matrix (five degrees of freedom, eight correspondences minimum) -- the
 *    constraint becomes linear in the translation direction and verifies
 *    suspects from three clean features. That is what keeps verification
 *    alive inside the starvation regime, where the essential-matrix path
 *    degenerates into unverified admission for everything.
 */
struct ImuMotionTracker {
    std::deque<std::pair<double, Eigen::Vector3d>> gyroWindow; ///< (stamp, w) samples of the energy window.
    Eigen::Matrix3d frameDelta = Eigen::Matrix3d::Identity();  ///< Gyro-integrated rotation since the last image.
    double windowSecs = 0.5; ///< Length of the motion-energy window.
    double lastStamp = -1.0;
    double gyroStd = 0.0;    ///< Windowed std of the gyro vector, rad/s.
    double accStd = 0.0;     ///< Windowed std of the accelerometer vector, m/s^2.

    void feed(const IMUVelocity& imu) {
        gyroWindow.emplace_back(imu.stamp, imu.gyr);
        while (!gyroWindow.empty() && imu.stamp - gyroWindow.front().first > windowSecs)
            gyroWindow.pop_front();
        if (gyroWindow.size() >= 2) {
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto& s : gyroWindow) mean += s.second;
            mean /= static_cast<double>(gyroWindow.size());
            double var = 0.0;
            for (const auto& s : gyroWindow) var += (s.second - mean).squaredNorm();
            gyroStd = std::sqrt(var / gyroWindow.size());
        }
        // Accelerometer magnitude window, kept separately from the vector
        // std: gravity dominates the vector, so its magnitude variance is the
        // usable tilt-dynamics signal.
        accWindow.emplace_back(imu.stamp, imu.acc.norm());
        while (!accWindow.empty() && imu.stamp - accWindow.front().first > windowSecs)
            accWindow.pop_front();
        if (accWindow.size() >= 2) {
            double mean = 0.0;
            for (const auto& s : accWindow) mean += s.second;
            mean /= static_cast<double>(accWindow.size());
            double var = 0.0;
            for (const auto& s : accWindow) var += (s.second - mean) * (s.second - mean);
            accStd = std::sqrt(var / accWindow.size());
        }
        if (lastStamp > 0.0) {
            const double dt = std::min(imu.stamp - lastStamp, 0.1);
            const Eigen::Vector3d w = imu.gyr * dt;
            const double angle = w.norm();
            if (angle > 1e-12)
                frameDelta = frameDelta * Eigen::AngleAxisd(angle, w.normalized()).toRotationMatrix();
        }
        lastStamp = imu.stamp;
    }

    /** @brief Return the gyro-integrated rotation since the previous image and reset it. */
    Eigen::Matrix3d captureFrameDelta() {
        Eigen::Matrix3d d = frameDelta;
        frameDelta.setIdentity();
        return d;
    }

  private:
    std::deque<std::pair<double, double>> accWindow; ///< (stamp, |acc|) samples of the energy window.
};

static ImuMotionTracker imuMotion;

/** @brief True if f falls inside b grown by pad pixels on every side.
 *
 * A negative pad selects scale-adaptive padding: 3.5 % of the box's longer side,
 * clamped to [1, 6] px. A fixed pad in pixels means something different for a
 * pedestrian filling the frame and one forty metres away, and the box outline
 * from a 320x320 detector is only accurate to a few pixels either way.
 */
static bool insidePadded(const GIFT::Feature& f, const cv::Rect2f& b, double pad) {
    const float p = (pad < 0.0) ? std::min(6.0f, std::max(1.0f, 0.035f * std::max(b.width, b.height)))
                                : static_cast<float>(pad);
    const cv::Rect2f pb(b.x - p, b.y - p, b.width + 2.0f * p, b.height + 2.0f * p);
    return pb.contains(f.camCoordinates);
}

static const char* gateRegimeName(int r) {
    switch (r) {
    case GateStats::Nominal:
        return "nominal";
    case GateStats::Starvation:
        return "starvation";
    case GateStats::Fallback:
        return "fallback";
    default:
        return "passthrough";
    }
}

static std::vector<GIFT::Feature> selectiveRejectSplit(std::vector<GIFT::Feature> clean,
    std::vector<GIFT::Feature> suspect, const GIFT::RansacParameters& params, std::mt19937& rng,
    double threshold);

/// Box-list form: no mask image is built at all (see PropagatedMaskSource::advance).
static std::vector<GIFT::Feature> selectiveDynamicRejection(const std::vector<GIFT::Feature>& features,
    const std::vector<cv::Rect2f>& boxes, const GIFT::RansacParameters& params, std::mt19937& rng,
    double threshold) {
    if (boxes.empty()) {
        gateStats.regime = GateStats::Passthrough;
        gateStats.nClean = static_cast<int>(features.size());
        return features;
    }
    std::vector<GIFT::Feature> clean, suspect;
    for (const auto& f : features) {
        bool in = false;
        for (const auto& b : boxes)
            if (b.contains(f.camCoordinates)) { in = true; break; }
        (in ? suspect : clean).push_back(f);
    }
    gateStats.regime = GateStats::Nominal;
    gateStats.nClean = static_cast<int>(clean.size());
    gateStats.nSuspect = static_cast<int>(suspect.size());
    return selectiveRejectSplit(std::move(clean), std::move(suspect), params, rng, threshold);
}

static std::vector<GIFT::Feature> selectiveDynamicRejection(const std::vector<GIFT::Feature>& features,
    const cv::Mat& keepMask, const GIFT::RansacParameters& params, std::mt19937& rng, double threshold) {
    if (keepMask.empty())
        return features;

    std::vector<GIFT::Feature> clean, suspect;
    for (const auto& f : features) {
        const int x = cvRound(f.camCoordinates.x), y = cvRound(f.camCoordinates.y);
        const bool inside = x >= 0 && y >= 0 && x < keepMask.cols && y < keepMask.rows;
        // keepMask is non-zero where the pixel is usable, so zero means masked.
        if (inside && keepMask.at<uchar>(y, x) == 0)
            suspect.push_back(f);
        else
            clean.push_back(f);
    }
    return selectiveRejectSplit(std::move(clean), std::move(suspect), params, rng, threshold);
}

/* Optional per-frame trace of what the gate decided, for the qualitative figure.
 *
 * Off unless --dyndetTrace is given, and it is a plain sink rather than a member
 * of the mask source because the decision is made in a free function that has no
 * access to either. One row per feature per frame:
 *
 *   stamp, x, y, set, residual, kept
 *
 * where set is 0 for clean and 1 for suspect. Clean features carry no residual
 * (they are never tested), so theirs is written as -1 rather than as 0, which
 * would be indistinguishable from a perfectly consistent measurement.
 */
static std::ofstream gateTrace;
static double gateTraceStamp = -1.0;

static void traceFeature(const GIFT::Feature& f, int set, double residual, bool kept) {
    if (!gateTrace.is_open())
        return;
    gateTrace << std::setprecision(17) << gateTraceStamp << "," << std::setprecision(7)
              << f.camCoordinates.x << "," << f.camCoordinates.y << "," << set << ","
              << residual << "," << (kept ? 1 : 0) << "\n";
}

static std::vector<GIFT::Feature> selectiveRejectSplit(std::vector<GIFT::Feature> clean,
    std::vector<GIFT::Feature> suspect, const GIFT::RansacParameters& params, std::mt19937& rng,
    double threshold) {
    /* Eight is the minimum correspondence count the essential matrix needs, so
     * below it there is no model to test against and every suspect feature is
     * admitted untested. Under the starvation floor this is not an edge case
     * but the normal path: the branch only runs when the background set is
     * smaller than N_safe, which is well under eight at any deployed setting.
     * Counted separately from a verified admission for exactly that reason. */
    if (suspect.empty() || clean.size() < 8) {
        for (const auto& f : clean)
            traceFeature(f, 0, -1.0, true);
        for (const auto& f : suspect)
            traceFeature(f, 1, -1.0, true); // fallback branch: everything admitted
        gateStats.nReadmittedUnverified += static_cast<int>(suspect.size());
        clean.insert(clean.end(), suspect.begin(), suspect.end());
        return clean; // nothing to test, or no trustworthy set to fit against
    }

    const std::vector<GIFT::Feature> inliers = GIFT::determineStaticWorldInliers(clean, params, rng);
    const Eigen::Matrix3T E = GIFT::fitEssentialMatrix(inliers.size() >= 8 ? inliers : clean);

    for (const auto& f : clean)
        traceFeature(f, 0, -1.0, true);

    std::vector<GIFT::Feature> kept = std::move(clean);
    for (const auto& f : suspect) {
        const cv::Point2f p2CV = f.camCoordinatesNorm();
        const Eigen::Vector3T p2(p2CV.x, p2CV.y, 1.0);
        const Eigen::Vector3T p1 = p2 - Eigen::Vector3T(f.opticalFlowNorm.x(), f.opticalFlowNorm.y(), 0.0);
        const double err = std::abs(static_cast<double>(p1.transpose() * E * p2));
        const bool admit = err < threshold;
        traceFeature(f, 1, err, admit);
        ++gateStats.nTested;
        if (admit) {
            ++gateStats.nReadmittedVerified;
            kept.push_back(f); // on the object, but consistent: still a usable measurement
        }
    }
    return kept;
}

static std::vector<GIFT::Feature> blanketDynamicRejection(const std::vector<GIFT::Feature>& features,
    const std::vector<cv::Rect2f>& boxes, double pad = 0.0, int minMotionRank = 0) {
    if (boxes.empty()) {
        gateStats.regime = GateStats::Passthrough;
        gateStats.nClean = static_cast<int>(features.size());
        return features;
    }
    // Hard masking is only safe while the previous visual update remains
    // observable.  If the update has already collapsed, deleting every boxed
    // feature makes the next update even weaker and turns a temporary detector
    // error into an irreversible estimator failure.  Keep the complete feature
    // set for this frame and let the filter recover; the normal blanket policy
    // is unchanged when the rank is healthy.
    if (minMotionRank > 0 && eqfDiagnostics.applied && eqfDiagnostics.motionRank < minMotionRank) {
        gateStats.regime = GateStats::Fallback;
        gateStats.nClean = static_cast<int>(features.size());
        for (const auto& f : features)
            traceFeature(f, 0, -1.0, true);
        return features;
    }
    gateStats.regime = GateStats::Nominal;
    std::vector<GIFT::Feature> kept;
    kept.reserve(features.size());
    for (const auto& f : features) {
        bool in = false;
        for (const auto& b : boxes) {
            if (insidePadded(f, b, pad)) {
                in = true;
                break;
            }
        }
        if (!in) {
            kept.push_back(f);
            traceFeature(f, 0, -1.0, true);
        } else {
            ++gateStats.nBoxRejected;
            traceFeature(f, 1, 999.0, false);
        }
    }
    gateStats.nClean = static_cast<int>(kept.size());
    return kept;
}

static std::vector<GIFT::Feature> differentiatedDynamicRejection(const std::vector<GIFT::Feature>& features,
    const std::vector<cv::Rect2f>& vehicleBoxes, const std::vector<cv::Rect2f>& personBoxes,
    const GIFT::RansacParameters& params, std::mt19937& rng, double threshold, double vehPad = 5.0) {
    gateStats.nVehBoxes = static_cast<int>(vehicleBoxes.size());
    gateStats.nPersonBoxes = static_cast<int>(personBoxes.size());
    if (vehicleBoxes.empty() && personBoxes.empty()) {
        gateStats.regime = GateStats::Passthrough;
        gateStats.nClean = static_cast<int>(features.size());
        return features;
    }

    std::vector<GIFT::Feature> clean, suspect;
    clean.reserve(features.size());
    suspect.reserve(features.size());

    for (const auto& f : features) {
        bool inVeh = false;
        for (const auto& b : vehicleBoxes) {
            if (insidePadded(f, b, vehPad)) {
                inVeh = true;
                break;
            }
        }
        if (inVeh) {
            traceFeature(f, 1, 999.0, false); // Hard vehicle rejection
            ++gateStats.nVehRejected;
            continue;
        }

        bool inPerson = false;
        for (const auto& b : personBoxes) {
            if (b.contains(f.camCoordinates)) {
                inPerson = true;
                break;
            }
        }
        if (inPerson) {
            suspect.push_back(f);
        } else {
            clean.push_back(f);
        }
    }

    gateStats.regime = GateStats::Nominal;
    gateStats.nClean = static_cast<int>(clean.size());
    gateStats.nSuspect = static_cast<int>(suspect.size());
    return selectiveRejectSplit(std::move(clean), std::move(suspect), params, rng, threshold);
}

/** @brief Verify suspect features against the static world using the
 * gyro-known inter-frame rotation.
 *
 * The essential-matrix test needs eight clean correspondences, which is
 * exactly what the starvation regime does not have -- below eight, the
 * selective path admits every suspect unverified. With the inter-frame
 * rotation known from the gyro (dRc), the static-world constraint loses the
 * rotation degrees of freedom and becomes linear in the translation
 * direction alone:
 *
 *   x2^T [t] x (dRc x1) = 0,  one row (y x x2)^T of a 3x3 system per feature,
 *
 * so three clean features fit the model and every suspect is verifiable.
 * The residual is the sine of the angle between the suspect's bearing and
 * its epipolar plane -- dimensionless, threshold ~0.02 rad.
 */
static std::map<int, double> gateNoiseWeights; ///< Per-feature soft weights produced by the gate.

static double softWeight(double residual, double tau) {
    // Use a robust, covariance-compatible weight.  Unlike the previous
    // tau/max(residual, ...) rule, this is strictly below one as soon as the
    // residual is non-zero, while remaining one for a perfect constraint.
    // Since R is scaled by 1/w^2 below, this is equivalent to inflating the
    // feature covariance by 1 + (residual/tau)^2.
    if (!std::isfinite(residual) || tau <= 0.0)
        return 0.1;
    const double ratio = residual / tau;
    const double w = 1.0 / std::sqrt(1.0 + ratio * ratio);
    return std::min(1.0, std::max(0.1, w));
}

/** @brief The adaptive policy, v4: one rule for both classes.
 *
 * When background features are abundant (>= N_safe) every boxed feature is
 * blanket-rejected -- the cheapest and safest treatment. When they are scarce,
 * boxed features are individually verified against the gyro-compensated
 * static-world constraint and only consistent ones are admitted. Either way,
 * every admitted feature carries a soft weight derived from its own residual,
 * so weakly consistent measurements inflate their measurement noise instead of
 * silently contaminating the update at full weight. The class differentiation
 * lives in the detector thresholds and padding of Section
 * \\ref{sec:method_adaptive}, not in a separate rejection rule.
 */
static std::vector<GIFT::Feature> adaptiveLegacyDynamicRejection(const std::vector<GIFT::Feature>& features,
    const std::vector<cv::Rect2f>& vehicleBoxesIn, const std::vector<cv::Rect2f>& personBoxesIn,
    const GIFT::RansacParameters& params, std::mt19937& rng, double threshold, double vehPad = 5.0,
    double personPad = 2.0, size_t minSafeClean = 4, int minMotionRank = 0,
    const Eigen::Matrix3d& dRc = Eigen::Matrix3d::Identity(), double imuTau = 0.02,
    bool imuVerify = false, bool softWeighting = true, bool fallbackVerify = false,
    double tauSoft = -1.0, double wSoft = 0.20) {
    // The admission test and the down-weighting scale are deliberately separate
    // knobs: admission should stay strict, while a looser soft scale keeps
    // mildly inconsistent geometry at usable weight instead of suppressing it.
    // The loose scale is applied in the NOMINAL regime only -- when background
    // is abundant the update is well conditioned and mild inconsistency is
    // better tolerated -- while the starvation and fallback regimes keep the
    // strict scale, because there contamination is what kills the sequence.
    const double tauW = (tauSoft > 0.0) ? tauSoft : imuTau;
    std::vector<cv::Rect2f> vehicleBoxes, personBoxes;
    if (ablation.useVehicleBoxes) {
        if (ablation.classDiff && ablation.vehicleReject)
            vehicleBoxes = vehicleBoxesIn;
        else
            personBoxes = vehicleBoxesIn; // demoted to the pedestrian rule
    }
    if (ablation.usePersonBoxes)
        personBoxes.insert(personBoxes.end(), personBoxesIn.begin(), personBoxesIn.end());

    gateStats.nVehBoxes = static_cast<int>(vehicleBoxes.size());
    gateStats.nPersonBoxes = static_cast<int>(personBoxes.size());

    std::vector<GIFT::Feature> clean, suspect;
    for (const auto& f : features) {
        bool inBox = false;
        for (const auto& b : vehicleBoxes)
            if (insidePadded(f, b, vehPad)) { inBox = true; break; }
        for (const auto& b : personBoxes)
            if (!inBox && insidePadded(f, b, personPad)) { inBox = true; break; }
        (inBox ? suspect : clean).push_back(f);
    }
    gateStats.nClean = static_cast<int>(clean.size());
    gateStats.nSuspect = static_cast<int>(suspect.size());

    // Fit the translation direction once from the clean set. With the
    // gyro-known rotation the constraint is linear in t, so three
    // non-collinear correspondences suffice -- where the eight-point fit
    // collapsed into blind admission, this fit keeps verification alive.
    bool fitOk = (static_cast<int>(clean.size()) >= 3);
    Eigen::Vector3d tHat = Eigen::Vector3d::Zero();
    if (fitOk) {
        Eigen::MatrixXd M(clean.size(), 3);
        for (size_t i = 0; i < clean.size(); ++i) {
            const cv::Point2f p2CV = clean[i].camCoordinatesNorm();
            const Eigen::Vector3d x2(p2CV.x, p2CV.y, 1.0);
            const Eigen::Vector3d x1Prev = x2 - Eigen::Vector3d(clean[i].opticalFlowNorm.x(),
                                                               clean[i].opticalFlowNorm.y(), 0.0);
            const Eigen::Vector3d y = dRc * x1Prev;
            M.row(i) = y.cross(x2);
        }
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
        if (svd.singularValues()(2) > 0.3 * svd.singularValues()(1)) {
            tHat = svd.matrixV().col(2);
        } else {
            fitOk = false; // collinear background: the direction is undetermined
        }
    }

    auto residualOf = [&](const GIFT::Feature& f) -> std::optional<double> {
        // A missing/degenerate model is not a zero-residual measurement.  It
        // is an unverified feature and must never be counted as verified.
        if (!fitOk) return std::nullopt;
        const cv::Point2f p2CV = f.camCoordinatesNorm();
        const Eigen::Vector3d x2(p2CV.x, p2CV.y, 1.0);
        const Eigen::Vector3d x1Prev = x2 - Eigen::Vector3d(f.opticalFlowNorm.x(),
                                                           f.opticalFlowNorm.y(), 0.0);
        const Eigen::Vector3d y = dRc * x1Prev;
        const Eigen::Vector3d n = tHat.cross(y);
        const double nn = n.norm();
        if (nn < 1e-9)
            return std::nullopt;
        const double residual = std::abs(x2.dot(n) / nn);
        return std::isfinite(residual) ? std::optional<double>(residual) : std::nullopt;
    };
    // Soft-rescue weight (plan Section 3.4): an untestable suspect in the
    // rescue regime keeps a bounded amount of visual information by inflating
    // its measurement covariance (R/w_soft^2 in the backend) rather than
    // entering either at full weight or not at all.
    const double fallbackWeight = std::min(1.0, std::max(0.01, wSoft));
    auto weightOf = [&](const GIFT::Feature& f, bool suspect, double tau) -> double {
        const auto residual = residualOf(f);
        if (suspect) {
            // Keep a bounded amount of visual information when the estimator
            // is already starved, but do not give an unverified suspect full
            // trust.  This is deliberately not called "verified" in the log.
            if (!residual || !imuVerify)
                return fallbackWeight;
            return softWeighting ? softWeight(*residual, tau) : 1.0;
        }
        // Clean features are allowed to receive a robust weight when a valid
        // model exists; this protects against detector/track outliers outside
        // a box without changing their membership in the clean set.
        return (softWeighting && imuVerify && residual) ? softWeight(*residual, tau) : 1.0;
    };

    // Observability guard: when the previous update constrained too few
    // ego-motion directions, rejection is suspended and every feature is kept
    // at its residual-derived weight -- losing observability costs more than
    // the contamination of keeping weakly consistent measurements.
    if (minMotionRank > 0 && eqfDiagnostics.applied && eqfDiagnostics.motionRank < minMotionRank) {
        gateStats.regime = GateStats::Fallback;
        if (!fitOk)
            gateStats.nModelFailures += static_cast<int>(suspect.size());
        // The guard exists to keep the feature set alive, not to admit every
        // suspect: with a valid gyro-compensated model, an inconsistent suspect
        // is rejected here exactly as the starvation branch would reject it,
        // while consistent and untestable suspects keep the rescue semantics.
        std::vector<GIFT::Feature> keptF;
        keptF.reserve(features.size());
        for (const auto& f : features) {
            const bool isSuspect = std::find_if(suspect.begin(), suspect.end(),
                [&](const GIFT::Feature& s) { return s.idNumber == f.idNumber; }) != suspect.end();
            const auto residual = residualOf(f);
            if (isSuspect && residual) {
                ++gateStats.nTested;
                if (fallbackVerify && *residual >= imuTau) {
                    const bool isVeh = [&] {
                        for (const auto& b : vehicleBoxes)
                            if (insidePadded(f, b, vehPad)) return true;
                        return false;
                    }();
                    if (isVeh)
                        ++gateStats.nVehRejected;
                    else
                        ++gateStats.nPedRejected;
                    traceFeature(f, 1, *residual, false);
                    continue;
                }
                ++gateStats.nReadmittedVerified;
            } else if (isSuspect) {
                ++gateStats.nReadmittedUnverified;
            }
            gateNoiseWeights[f.idNumber] = weightOf(f, isSuspect, imuTau);
            traceFeature(f, isSuspect ? 1 : 0, residual ? *residual : -1.0, true);
            keptF.push_back(f);
        }
        return keptF;
    }

    if (clean.size() >= minSafeClean) {
        // Nominal: enough background -- blanket rejection of every boxed
        // feature, cheapest and safest for the update.
        gateStats.regime = GateStats::Nominal;
        for (const auto& f : suspect) {
            const bool isVeh = [&] {
                for (const auto& b : vehicleBoxes)
                    if (insidePadded(f, b, vehPad)) return true;
                return false;
            }();
            if (isVeh) {
                ++gateStats.nVehRejected;
                traceFeature(f, 1, 999.0, false);
            } else {
                ++gateStats.nPedRejected;
                traceFeature(f, 1, 999.0, false);
            }
        }
        for (const auto& f : clean) {
            gateNoiseWeights[f.idNumber] = weightOf(f, false, tauW);
            traceFeature(f, 0, -1.0, true);
        }
        return clean;
    }

    // Starvation: individually verify every suspect against the
    // gyro-compensated constraint and keep the consistent ones at their
    // residual-derived weight.
    gateStats.regime = GateStats::Starvation;
    std::vector<GIFT::Feature> kept = std::move(clean);
    for (const auto& f : suspect) {
        const auto residual = residualOf(f);
        if (residual)
            ++gateStats.nTested;
        const bool isVeh = [&] {
            for (const auto& b : vehicleBoxes)
                if (insidePadded(f, b, vehPad)) return true;
            return false;
        }();
        if (residual && imuVerify && *residual < imuTau) {
            ++gateStats.nReadmittedVerified;
            gateNoiseWeights[f.idNumber] = softWeighting ? softWeight(*residual, imuTau) : 1.0;
            traceFeature(f, 1, *residual, true);
            kept.push_back(f);
        } else {
            if (!residual || !imuVerify) {
                ++gateStats.nReadmittedUnverified;
                if (!residual)
                    ++gateStats.nModelFailures;
                gateNoiseWeights[f.idNumber] = weightOf(f, true, imuTau);
                traceFeature(f, 1, residual ? *residual : -1.0, true);
                kept.push_back(f);
            } else {
                if (isVeh) ++gateStats.nVehRejected; else ++gateStats.nPedRejected;
                traceFeature(f, 1, *residual, false);
            }
        }
    }
    for (const auto& f : clean) {
        gateNoiseWeights[f.idNumber] = weightOf(f, false, imuTau);
        traceFeature(f, 0, -1.0, true);
    }
    return kept;
}

static double adaptiveClamp01(double x) {
    return std::min(1.0, std::max(0.0, x));
}

/** A cheap, deterministic per-feature information proxy.
 *
 * The full EqF information matrix is only available after the measurement is
 * assembled. The gate nevertheless needs a pre-update decision. This proxy
 * favours features with useful parallax and image leverage while retaining a
 * non-zero floor for central/low-motion tracks. The actual post-update motion
 * rank remains the hard observability guard and is logged for validation.
 */
static double adaptiveFeatureInformation(const GIFT::Feature& f) {
    const cv::Point2f p = f.camCoordinatesNorm();
    const double leverage = adaptiveClamp01(std::sqrt(p.x * p.x + p.y * p.y) / 1.15);
    const double parallax = adaptiveClamp01(f.opticalFlowNorm.norm() / 0.08);
    return std::min(1.0, 0.25 + 0.45 * leverage + 0.30 * parallax);
}

/** Risk-aware, temporally adaptive feature allocation.
 *
 * Unlike the legacy policy, a boxed feature is not treated as guilty merely
 * because it is inside a detector rectangle. Risk is accumulated over the
 * feature track, and the information proxy decides whether a risky feature is
 * removed or retained with inflated covariance. When the previous EqF update
 * was weak, high-information suspect features are retained at a bounded weight
 * so dynamic rejection cannot starve the estimator.
 */
static std::vector<GIFT::Feature> adaptiveDynamicRejection(const std::vector<GIFT::Feature>& features,
    const std::vector<cv::Rect2f>& vehicleBoxesIn, const std::vector<cv::Rect2f>& personBoxesIn,
    const GIFT::RansacParameters&, std::mt19937&, double, double vehPad = 5.0,
    double personPad = 2.0, size_t minSafeClean = 8, int minMotionRank = 0,
    const Eigen::Matrix3d& dRc = Eigen::Matrix3d::Identity(), double imuTau = 0.02,
    bool imuVerify = false, bool softWeighting = true, bool fallbackVerify = false,
    double tauSoft = -1.0, double riskAlpha = 0.35, double riskHigh = 0.70,
    double riskLow = 0.35, int riskRise = 2, int riskFall = 5,
    double weightFloor = 0.20, double informationFloor = 0.35) {
    ++adaptiveFrame;
    for (auto it = adaptiveFeatureTracks.begin(); it != adaptiveFeatureTracks.end();) {
        if (adaptiveFrame > it->second.lastFrame + 30)
            it = adaptiveFeatureTracks.erase(it);
        else
            ++it;
    }

    std::vector<cv::Rect2f> vehicleBoxes, personBoxes;
    if (ablation.useVehicleBoxes) {
        if (ablation.classDiff && ablation.vehicleReject)
            vehicleBoxes = vehicleBoxesIn;
        else
            personBoxes = vehicleBoxesIn;
    }
    if (ablation.usePersonBoxes)
        personBoxes.insert(personBoxes.end(), personBoxesIn.begin(), personBoxesIn.end());

    gateStats.nVehBoxes = static_cast<int>(vehicleBoxes.size());
    gateStats.nPersonBoxes = static_cast<int>(personBoxes.size());

    auto classIsVehicle = [&](const GIFT::Feature& f) {
        for (const auto& b : vehicleBoxes)
            if (insidePadded(f, b, vehPad))
                return true;
        return false;
    };
    auto isSuspect = [&](const GIFT::Feature& f) {
        for (const auto& b : vehicleBoxes)
            if (insidePadded(f, b, vehPad))
                return true;
        for (const auto& b : personBoxes)
            if (insidePadded(f, b, personPad))
                return true;
        return false;
    };

    std::vector<GIFT::Feature> clean, suspect;
    clean.reserve(features.size());
    suspect.reserve(features.size());
    for (const auto& f : features)
        (isSuspect(f) ? suspect : clean).push_back(f);
    gateStats.nClean = static_cast<int>(clean.size());
    gateStats.nSuspect = static_cast<int>(suspect.size());

    const bool observabilityWeak = ablation.starvationFloor &&
        ((minSafeClean > 0 && clean.size() < minSafeClean) ||
         (minMotionRank > 0 && eqfDiagnostics.applied && eqfDiagnostics.motionRank < minMotionRank));
    gateStats.regime = observabilityWeak ?
        (clean.size() < minSafeClean ? GateStats::Starvation : GateStats::Fallback) : GateStats::Nominal;

    // Detector membership is the only reason to run the adaptive policy. On
    // ordinary frames there may be no propagated box feature at all; return
    // immediately so adaptive mode follows the same zero-overhead path as
    // baseline instead of fitting a motion model and creating 1.0 weights for
    // every clean feature.
    if (suspect.empty())
        return features;

    // Estimate the static-world epipolar direction from clean tracks. The
    // gyro-known rotation reduces this to a translation-direction fit and
    // needs only three non-collinear correspondences. This gives the risk
    // state a geometric cue that is invariant to ordinary camera rotation;
    // raw optical flow alone would confuse a fast yaw with object motion.
    bool fitOk = clean.size() >= 3;
    Eigen::Vector3d tHat = Eigen::Vector3d::Zero();
    if (fitOk) {
        Eigen::MatrixXd M(clean.size(), 3);
        for (size_t i = 0; i < clean.size(); ++i) {
            const cv::Point2f p2CV = clean[i].camCoordinatesNorm();
            const Eigen::Vector3d x2(p2CV.x, p2CV.y, 1.0);
            const Eigen::Vector3d x1Prev = x2 - Eigen::Vector3d(
                clean[i].opticalFlowNorm.x(), clean[i].opticalFlowNorm.y(), 0.0);
            M.row(i) = (dRc * x1Prev).cross(x2);
        }
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
        if (svd.singularValues()(2) > 0.3 * svd.singularValues()(1))
            tHat = svd.matrixV().col(2);
        else
            fitOk = false;
    }

    auto residualOf = [&](const GIFT::Feature& f) -> std::optional<double> {
        if (!fitOk)
            return std::nullopt;
        const cv::Point2f p2CV = f.camCoordinatesNorm();
        const Eigen::Vector3d x2(p2CV.x, p2CV.y, 1.0);
        const Eigen::Vector3d x1Prev = x2 - Eigen::Vector3d(
            f.opticalFlowNorm.x(), f.opticalFlowNorm.y(), 0.0);
        const Eigen::Vector3d y = dRc * x1Prev;
        const Eigen::Vector3d n = tHat.cross(y);
        const double nn = n.norm();
        if (nn < 1e-9)
            return std::nullopt;
        const double residual = std::abs(x2.dot(n) / nn);
        return std::isfinite(residual) ? std::optional<double>(residual) : std::nullopt;
    };

    const double tauWeight = tauSoft > 0.0 ? tauSoft : imuTau;
    const int highFrames = std::max(1, riskRise);
    const int lowFrames = std::max(1, riskFall);
    const double alpha = adaptiveClamp01(riskAlpha);
    const double loWeight = std::min(1.0, std::max(0.05, weightFloor));
    const double infoFloor = adaptiveClamp01(informationFloor);
    double riskSum = 0.0, infoSum = 0.0, weightSum = 0.0;
    int considered = 0, weighted = 0;
    std::set<int> rejectedIds;

    for (const auto& f : features) {
        const bool suspectFeature = isSuspect(f);
        const bool vehicleFeature = suspectFeature && classIsVehicle(f);
        const double info = adaptiveFeatureInformation(f);
        const double pseudoResidual = f.opticalFlowNorm.norm();
        const double motionEvidence = adaptiveClamp01(pseudoResidual / 0.08);
        const auto geometricResidual = residualOf(f);
        const double geometricEvidence = geometricResidual
            ? adaptiveClamp01(*geometricResidual / std::max(imuTau, 1e-6))
            : motionEvidence;
        // Detector membership is a prior, not a verdict. The temporal risk is
        // driven mainly by rotation-compensated geometric inconsistency, with
        // flow retained as a fallback when the clean set cannot fit a model.
        const double evidence = suspectFeature
            ? (0.18 + 0.62 * geometricEvidence + 0.20 * motionEvidence)
            : (0.03 + 0.08 * geometricEvidence + 0.04 * motionEvidence);

        AdaptiveFeatureTrack& track = adaptiveFeatureTracks[f.idNumber];
        track.lastFrame = adaptiveFrame;
        track.risk = (1.0 - alpha) * track.risk + alpha * evidence;
        if (track.risk >= riskHigh) {
            ++track.highEvidence;
            track.lowEvidence = 0;
        } else if (track.risk <= riskLow) {
            ++track.lowEvidence;
            track.highEvidence = 0;
        } else {
            track.highEvidence = 0;
            track.lowEvidence = 0;
        }
        if (track.highEvidence >= highFrames)
            track.state = AdaptiveFeatureState::Moving;
        else if (track.lowEvidence >= lowFrames)
            track.state = AdaptiveFeatureState::Static;
        else if (track.state == AdaptiveFeatureState::Unknown)
            track.state = AdaptiveFeatureState::SemiStatic;

        const bool moving = track.state == AdaptiveFeatureState::Moving || track.risk >= riskHigh;
        if (moving)
            ++gateStats.nTemporalMoving;

        double weight = 1.0;
        bool reject = false;
        if (suspectFeature) {
            // Low-information dynamic observations are the safest removals;
            // high-information observations survive as bounded measurements.
            reject = moving && info < infoFloor &&
                (!observabilityWeak || (fallbackVerify && geometricResidual &&
                                         *geometricResidual >= imuTau));
            if (!reject) {
                const double riskWeight = 1.0 - 0.72 * track.risk;
                const double infoWeight = 0.65 + 0.35 * info;
                weight = std::max(loWeight, std::min(1.0, riskWeight * infoWeight));
                if (imuVerify && softWeighting && geometricResidual)
                    weight = std::min(weight, softWeight(*geometricResidual, tauWeight));
                if (!imuVerify || !softWeighting)
                    weight = std::min(weight, moving ? 0.55 : 0.85);
                if (observabilityWeak)
                    weight = std::max(weight, std::min(0.45, 1.0 - 0.5 * track.risk));
                ++weighted;
                ++gateStats.nRiskWeighted;
            }
        }

        riskSum += track.risk;
        infoSum += info;
        ++considered;
        if (reject) {
            ++gateStats.nRiskRemoved;
            rejectedIds.insert(f.idNumber);
            if (vehicleFeature)
                ++gateStats.nVehRejected;
            else
                ++gateStats.nPedRejected;
            traceFeature(f, 1, 999.0, false);
            continue;
        }

        if (suspectFeature) {
            if (geometricResidual && *geometricResidual < imuTau)
                ++gateStats.nReadmittedVerified;
            else
                ++gateStats.nReadmittedUnverified;
            if (geometricResidual)
                ++gateStats.nTested;
            traceFeature(f, 1, geometricResidual ? *geometricResidual : motionEvidence, true);
        } else {
            traceFeature(f, 0, -1.0, true);
        }
        // A unit weight is the backend default. Do not materialise it in the
        // map: doing so would force VIOFilter through the weighted covariance
        // construction even when adaptive has no effective weighting.
        if (weight < 0.999999)
            gateNoiseWeights[f.idNumber] = weight;
        weightSum += weight;
    }

    gateStats.riskMean = considered ? riskSum / considered : 0.0;
    gateStats.informationMean = considered ? infoSum / considered : 0.0;
    gateStats.weightMean = (considered - gateStats.nRiskRemoved) > 0
        ? weightSum / (considered - gateStats.nRiskRemoved) : 1.0;
    (void)weighted;
    return [&] {
        std::vector<GIFT::Feature> kept;
        kept.reserve(features.size() - std::min<size_t>(features.size(), gateStats.nRiskRemoved));
        for (const auto& f : features)
            if (!rejectedIds.count(f.idNumber))
                kept.push_back(f);
        return kept;
    }();
}

/** Percentile (linear interpolation, nearest-rank fallback for tiny inputs) of a
 * small rolling window. Called at most once per camera frame on windows of a
 * few dozen entries, so an O(n log n) copy-sort per call is negligible. */
static double percentileOf(const std::deque<int>& window, double p) {
    if (window.empty()) return 0.0;
    std::vector<int> sorted(window.begin(), window.end());
    std::sort(sorted.begin(), sorted.end());
    const double rank = (p / 100.0) * (sorted.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(rank));
    const size_t hi = static_cast<size_t>(std::ceil(rank));
    if (lo == hi) return static_cast<double>(sorted[lo]);
    const double frac = rank - lo;
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

int main(int argc, char const* argv[]) {
    // TODO: Add options for writing state and images
    argparse::ArgumentParser program("EqVIO standalone");
    program.add_argument("-d", "--display")
        .help("Display a live plot of the system state")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--mode").default_value(std::string("asl")).action([](const std::string& value) {
        static const std::vector<std::string> choices = {"asl", "uzhfpv", "anu", "ros", "hilti"};
        if (std::find(choices.begin(), choices.end(), value) != choices.end()) {
            return value;
        }
        throw std::invalid_argument("Invalid mode requested.");
    });
    program.add_argument("--simvis")
        .help("Simulate the vision measurements from ground truth.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--simimu")
        .help("Simulate the IMU measurements from ground truth.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("dataset").help("The name of the dataset.");
    program.add_argument("config").help("The yaml file with the filter and feature tracker configuration.");
    program.add_argument("-o", "--output")
        .help("Directory for storing the filter output files.")
        .default_value(std::string("-"));
    program.add_argument("-c", "--camera")
        .help("File containing camera intrinsics and extrinsics.")
        .default_value(std::string("-"));
    program.add_argument("--timing")
        .help("Record the processing time information.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--start")
        .help("Start processing a number of seconds after the start of the data.")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--stop")
        .help("Stop processing a number of seconds after the start of the data.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--limitRate")
        .help("Limit the playback rate of the data. Default 0.0 (no limit).")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--pacedReplay")
        .help("Replay camera and IMU on their timestamp schedule; record scheduled-arrival to output latency.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--dynmask")
        .help("Directory of per-frame dynamic-object masks named <timestamp_ns>.png, "
              "255 where dynamic. Features there are neither detected nor tracked.")
        .default_value(std::string(""));
    program.add_argument("--dynmaskMode")
        .help("How to use dynamic masking: 'blanket', 'selective', 'differentiated', 'adaptive', "
              "'adaptive_legacy', or 'opdg'. 'opdg' is the Observability-Preserving Dynamic Gating "
              "policy of plan Section 3.4: the same tiered gate as adaptive_legacy with the "
              "gyro-compensated geometric verification always on and the soft-rescue weight from "
              "--dynmaskSoftRescue.")
        .default_value(std::string("blanket"));
    program.add_argument("--dynmaskMinRank")
        .help("Suspend pedestrian rejection whenever the previous visual update constrained "
              "fewer than this many of the nine ego-motion directions (attitude, position, "
              "velocity). 0 disables the guard. The rank is measured by the filter and costs "
              "nothing to read; it detects a poorly conditioned update that a feature count "
              "cannot. Vehicle rejection is never suspended.")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("--dynmaskMinClean")
        .help("Minimum number of clean background features to allow blanket masking on pedestrians (default 4).")
        .default_value(4)
        .scan<'i', int>();
    program.add_argument("--dyndetVehConf")
        .help("Minimum detector confidence for vehicle classes (default 0.45).")
        .default_value(0.45)
        .scan<'g', double>();
    program.add_argument("--dynmaskVehPad")
        .help("Padding in pixels around vehicle bounding boxes for dynamic rejection.")
        .default_value(5.0)
        .scan<'g', double>();
    program.add_argument("--dynmaskPersonPad")
        .help("Padding in pixels around person bounding boxes for dynamic rejection (-1.0 for auto scale-aware padding).")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--dyndet")
        .help("Detector CSV (image,class_id,class_name,confidence,x,y,w,h). Boxes are "
              "refreshed every --dyndetInterval frames and carried across the gap by the "
              "tracker's own optical flow, so the detector need not run every frame.")
        .default_value(std::string(""));
    program.add_argument("--dyndetLive")
        .help("Live detector endpoint, currently udp://127.0.0.1:<port>. The estimator polls "
              "it non-blocking and accepts packets while continuing every frame; use this "
              "with yolo/live_detect.py instead of --dyndet.")
        .default_value(std::string(""));
    program.add_argument("--dyndetInterval")
        .help("Frames between detector invocations when using --dyndet.")
        .default_value(1)
        .scan<'i', int>();
    program.add_argument("--dyndetDelayMs")
        .help("Discrete-event emulation: accelerator latency T_lat in ms between the "
              "detector's source frame and the first estimator frame that may see its "
              "boxes (RTX ~4, KPU ~50, C908 CPU ~284). 0 adopts detections on their "
              "source frame, which is the legacy synchronous behaviour.")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--dyndetJitterMs")
        .help("Discrete-event emulation: half-width of the uniform +/- latency jitter "
              "in ms, drawn from the --seed PRNG so a scenario is reproducible.")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--dyndetDropRate")
        .help("Discrete-event emulation: probability that a detection packet never "
              "arrives (dropped DMA transfer / lost queue slot), drawn from the --seed "
              "PRNG. The previous boxes keep propagating until the TTL horizon.")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--dyndetMaxAgeFrames")
        .help("TTL horizon: active boxes whose age from their detection's SOURCE frame "
              "exceeds this many frames are expired and discarded, so a lost detection "
              "cannot mask the wrong pixels indefinitely. 0 disables expiry.")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("--dyndetBlocking")
        .help("Ablation B10: run the detector synchronously -- the estimator thread stalls "
              "for T_lat on every detection frame and gets current boxes, instead of "
              "consuming stale boxes asynchronously. Breaches the 20 Hz deadline by "
              "construction; use only for the queue-scheduling latency study.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--dyndetPadBase")
        .help("Freshness-Aware Dynamic Masking: fixed base padding in px applied to every "
              "presented box. A negative value (the default) disables FADM and presents "
              "the raw propagated rectangles.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--dyndetPadAge")
        .help("FADM: boundary dilation per millisecond of detection age (alpha_age), so a "
              "box sitting in the accelerator queue for T ms grows by alpha_age*T px per side.")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--dyndetPadMotion")
        .help("FADM: boundary dilation per pixel of the box's own per-frame displacement "
              "(beta_motion), expanding faster with objects that move faster in the image.")
        .default_value(0.0)
        .scan<'g', double>();
    program.add_argument("--dyndetPadMax")
        .help("FADM: ceiling on the total per-side dilation in px.")
        .default_value(60.0)
        .scan<'g', double>();
    program.add_argument("--dyndetGrow")
        .help("Pixels added per side per propagated frame.")
        .default_value(1.5)
        .scan<'g', double>();
    program.add_argument("--dyndetTrack")
        .help("How to move a box between detections. 'features' reuses the displacements "
              "GIFT already computed (0.04 ms/frame, but follows the camera when a box holds "
              "no features); 'roi' seeds points once per detection and tracks them with one "
              "KLT call for all boxes (4.6 ms); 'redetect' finds fresh points inside every box "
              "every frame (8.4 ms, most accurate); 'global' carries every box by the global "
              "ego-motion median only (ablation B4); 'internal_only' moves a box only by its "
              "own internal features and freezes it when it has none (ablation B5).")
        .default_value(std::string("features"));
    program.add_argument("--dyndetConf")
        .help("Minimum detector confidence.")
        .default_value(0.25)
        .scan<'g', double>();
    program.add_argument("--dynmaskSelThresh")
        .help("Epipolar residual above which a masked feature is rejected in selective "
              "mode. Default -1 uses GIFT's own ransacParams:inlierThreshold.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--dynmaskFallbackVerify")
        .help("In the observability-guard fallback, verify suspects against the "
              "gyro-compensated epipolar constraint and reject inconsistent ones instead "
              "of readmitting every suspect at the fallback weight.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--dynmaskSoftRescue")
        .help("Soft-rescue weight w_soft of the OPDG policy: suspects admitted in the "
              "starvation/fallback regimes without a usable geometric test carry this "
              "measurement weight (the backend inflates R by 1/w_soft^2). Default 0.20.")
        .default_value(0.20)
        .scan<'g', double>();
    program.add_argument("--gateTauImu")
        .help("Residual threshold (rad) of the gyro-compensated static-world test. Negative "
              "keeps the YAML gateTauImu.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--gateTauSoft")
        .help("Soft-weight scale (rad) for the residual-derived measurement weights; looser "
              "than the verification threshold it de-weights outliers less aggressively. "
              "Negative keeps it equal to gateTauImu.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--gateSoftWeight")
        .help("Override main:gateSoftWeight from YAML (1 enables, 0 disables residual soft "
              "weighting). Negative keeps the YAML value.")
        .default_value(-1)
        .scan<'i', int>();
    program.add_argument("--adaptiveRiskAlpha")
        .help("EMA update rate for the temporal adaptive risk state.")
        .default_value(0.35)
        .scan<'g', double>();
    program.add_argument("--adaptiveRiskHigh")
        .help("Risk threshold for the moving temporal state.")
        .default_value(0.70)
        .scan<'g', double>();
    program.add_argument("--adaptiveRiskLow")
        .help("Risk threshold for the static temporal state.")
        .default_value(0.35)
        .scan<'g', double>();
    program.add_argument("--adaptiveRiskRise")
        .help("High-risk observations required to enter moving state.")
        .default_value(2)
        .scan<'i', int>();
    program.add_argument("--adaptiveRiskFall")
        .help("Low-risk observations required to enter static state.")
        .default_value(5)
        .scan<'i', int>();
    program.add_argument("--adaptiveWeightFloor")
        .help("Minimum covariance weight for retained suspect features.")
        .default_value(0.20)
        .scan<'g', double>();
    program.add_argument("--adaptiveInfoFloor")
        .help("Information score below which high-risk features are removed.")
        .default_value(0.35)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovariance")
        .help("Global, frame-level measurement covariance scaling R_k = alpha_k * R_0, "
              "alpha_k a function of the PREVIOUS frame's innovation RMS and n_meas_used: "
              "R inflates (visual measurements trusted less) after a frame with a large "
              "residual or few surviving measurements, deflates (trusted more) after a "
              "clean, well-supported frame. Distinct from --adaptiveWeightFloor/InfoFloor, "
              "which are PER-FEATURE weights from the dynamic-object gate; this is a single "
              "scalar applied uniformly to every feature this frame, multiplied on top of "
              "whatever the gate already set. Off by default (alpha_k == 1, unchanged R).")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveCovarianceK1")
        .help("Weight on the normalised previous-frame innovation RMS term in alpha_k.")
        .default_value(0.5)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceK2")
        .help("Weight on the previous-frame measurement-shortfall term in alpha_k.")
        .default_value(0.5)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceERef")
        .help("Reference innovation RMS that normalises the e_k/e_ref term in alpha_k.")
        .default_value(0.5)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceNTarget")
        .help("Target n_meas_used that normalises the (1 - N/N_target) term in alpha_k; "
              "negative uses the configured maxFeatures.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceAlphaMin")
        .help("Lower clip on alpha_k (never trust vision more than this factor).")
        .default_value(0.3)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceAlphaMax")
        .help("Upper clip on alpha_k (never distrust vision more than this factor).")
        .default_value(8.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceAlphaSlew")
        .help("Max change in the APPLIED alpha_k per frame (rate limit). A single bad-innovation "
              "frame otherwise jumps alpha straight to alphaMax, which under-corrects the very "
              "drift that caused the spike, producing a bigger spike next frame -- a positive "
              "feedback loop that can run away into divergence. Negative disables the limiter "
              "(the raw clipped target is applied directly, previous behaviour).")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceAutoNTarget")
        .help("Self-calibrate N_target from a rolling percentile of this run's own "
              "n_meas_used history instead of the fixed --adaptiveCovarianceNTarget. "
              "Fixes chronic feature-scarce scenes (e.g. a tight featureDist on a "
              "narrow image) being misread as measurement corruption: a scene that "
              "never reaches the YAML maxFeatures otherwise has (1 - N/N_target) "
              "pinned near its ceiling every frame, inflating alpha_k continuously.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveCovarianceNTargetWindow")
        .help("Rolling window size (frames) for the self-calibrated N_target.")
        .default_value(50)
        .scan<'i', int>();
    program.add_argument("--adaptiveCovarianceNTargetPercentile")
        .help("Percentile of the rolling n_meas_used window used as N_target.")
        .default_value(70.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveCovarianceNTargetTracksBudget")
        .help("When --adaptiveBudget is also active, set N_target each frame from the budget's "
              "OWN current maxFeatures cap (scaled by --adaptiveCovarianceNTargetBudgetFraction), "
              "instead of a scene-wide history percentile. Without this, an intentional budget "
              "cut (e.g. maxFeatures dropped to 3) reads to the covariance term as unexplained "
              "measurement scarcity, and the two adaptive mechanisms fight each other: the "
              "budget's own designed-in low-N phases get treated as corruption and inflate "
              "alpha_k right when the filter can least afford under-correction. Overrides "
              "--adaptiveCovarianceAutoNTarget when both are set.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveCovarianceNTargetBudgetFraction")
        .help("Fraction of the budget's current maxFeatures cap used as N_target when "
              "--adaptiveCovarianceNTargetTracksBudget is set (outlier rejection means "
              "n_meas_used rarely reaches the cap even in healthy tracking).")
        .default_value(0.6)
        .scan<'g', double>();
    program.add_argument("--dyndetTrace")
        .help("Write a per-frame trace of the gate's decisions to this file: one row per "
              "feature, 'stamp,x,y,set,residual,kept', with set 0 clean and 1 suspect. "
              "Diagnostic only -- it is what the qualitative figure is drawn from, and it "
              "costs an unbuffered write per feature, so it is off by default.")
        .default_value(std::string(""));
    program.add_argument("--dynmaskDilate")
        .help("Dilate each dynamic mask by this many pixels before use.")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("--featureDistOverride")
        .help("Host sensitivity override for GIFT feature spacing; negative keeps YAML value.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--maxFeaturesOverride")
        .help("Host sensitivity override for GIFT maximum tracked features; negative keeps YAML value.")
        .default_value(-1)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudget")
        .help("Rank-driven frontend budget (mechanism N1): empty (off) or 'rank'. With 'rank' "
              "the GIFT feature spacing follows the motion rank of the previous vision "
              "update -- dense while the smoothed rank sits in [rankLow, rankHigh), sparse "
              "outside it. Both ends of the band are sparse on purpose: rank 0 means the "
              "frame saw no parallax and more features cannot help, rank 9 means it is "
              "already fully constrained and more features are not needed. See "
              "notes/research-plan.md RQ2.")
        .default_value(std::string(""));
    program.add_argument("--adaptiveBudgetDense")
        .help("Feature spacing while the smoothed rank is inside the band; negative means "
              "half the configured featureDist.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetSparse")
        .help("Feature spacing outside the band; negative keeps the configured featureDist, "
              "so an adaptive run starts from the proven operating point.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetRankLow")
        .help("Lower edge of the dense band, on the smoothed motion rank.")
        .default_value(1.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetRankHigh")
        .help("Upper edge of the dense band, exclusive; must exceed rankLow.")
        .default_value(9.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetSmooth")
        .help("EMA weight for the motion rank in [0,1]; 1 uses the raw per-frame rank.")
        .default_value(0.3)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetMaxLow")
        .help("Detection-cadence mode: maxFeatures while the smoothed rank is OUTSIDE "
              "the band, i.e. when the update is either degenerate (rank 0, no parallax "
              "to resolve) or already fully constrained (rank 9). Lower means GIFT's "
              "skip test (features > featureSearchThreshold * maxFeatures) fires more "
              "often, so the whole-image detector runs less often.")
        .default_value(8)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetMaxHigh")
        .help("Detection-cadence mode: maxFeatures INSIDE the band, where the geometry "
              "is weak but usable and replenishing features actually helps. Negative "
              "keeps the configured maxFeatures.")
        .default_value(-1)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetMeasFloor")
        .help("Starvation-guard mode: the number of measurements the filter must "
              "keep using. Below it the whole-image detector is re-enabled on the "
              "very next frame, without smoothing or hold, because this guard "
              "exists to catch a collapse rather than to save compute.")
        .default_value(5)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetHold")
        .help("Minimum frames between two budget switches; stops the policy flapping. "
              "Ignored (as a floor, not a fixed value) when --adaptiveBudgetAutoHold is set.")
        .default_value(5)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetAutoFloor")
        .help("Starvation-guard mode only: replace the fixed --adaptiveBudgetMeasFloor "
              "with a percentile of the estimator's own recent n_meas_used history, so "
              "the same command line self-calibrates to whatever measurement scale the "
              "current resolution/calibration naturally yields, instead of needing a "
              "hand-picked absolute floor per scenario.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveBudgetAutoFloorWindow")
        .help("Rolling window (frames) of n_meas_used history used by --adaptiveBudgetAutoFloor.")
        .default_value(50)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetAutoFloorPercentile")
        .help("Percentile (0-100) of the rolling n_meas_used window used as the "
              "self-calibrated starvation floor.")
        .default_value(40.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetAutoHold")
        .help("Starvation-guard mode only: grow the effective hold time above "
              "--adaptiveBudgetHold when the policy has been switching state frequently "
              "in its own recent history, damping the flap-then-diverge failure mode "
              "found on V2_03_difficult (57%% of frames switching state) without paying "
              "the fixed cost of a permanently long hold on sequences that never flap.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveBudgetAutoHoldWarmup")
        .help("Use base hold until half the switch window is observed; exclude the startup cap transition from switching history.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveBudgetAutoHoldWindow")
        .help("Rolling window (frames) of switch history used by --adaptiveBudgetAutoHold.")
        .default_value(50)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetAutoHoldGain")
        .help("Extra hold frames per unit recent switch-rate (switches/window), added on "
              "top of --adaptiveBudgetHold, when --adaptiveBudgetAutoHold is set.")
        .default_value(40.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetAutoCeiling")
        .help("Starvation-guard mode only: replace the fixed --adaptiveBudgetMaxHigh with "
              "a value calibrated from periodic uncapped probes, closing the circularity "
              "that --adaptiveBudgetAutoFloor alone cannot: a floor computed only from "
              "n_meas_used history that is itself bounded by maxHigh can never learn that "
              "the scene needs MORE than maxHigh, because it has never been allowed to try. "
              "Every --adaptiveBudgetProbePeriod frames, maxFeatures is lifted to the "
              "estimator's configured (YAML) value for --adaptiveBudgetProbeDuration frames; "
              "the n_meas_used observed then feeds a rolling history that sets maxHigh via "
              "--adaptiveBudgetProbePercentile.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--adaptiveBudgetProbePeriod")
        .help("Frames between ceiling probes.")
        .default_value(100)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetProbeDuration")
        .help("Frames per probe with maxFeatures uncapped.")
        .default_value(2)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetProbePercentile")
        .help("Percentile (0-100) of the rolling probe-history used as the self-calibrated "
              "ceiling (maxHigh).")
        .default_value(80.0)
        .scan<'g', double>();
    program.add_argument("--adaptiveBudgetProbeHistorySize")
        .help("Number of completed probes kept in the rolling ceiling-history window.")
        .default_value(10)
        .scan<'i', int>();
    program.add_argument("--adaptiveBudgetLowFraction")
        .help("With --adaptiveBudgetAutoCeiling, also scale the aggressive floor (maxLow) "
              "as this fraction of the calibrated ceiling (maxHigh), instead of leaving it "
              "at the fixed --adaptiveBudgetMaxLow. Found necessary empirically: a fixed "
              "maxLow=3 against a ceiling the probe calibrates up to ~36 (quarter-res "
              "MH_02_easy) produces an occasional 36->3 swing large enough to diverge the "
              "filter on its own, even though the switch-rate staying low means "
              "--adaptiveBudgetAutoHold does not see it as flapping. A negative value keeps "
              "the fixed --adaptiveBudgetMaxLow (the old, resolution-specific behaviour).")
        .default_value(0.25)
        .scan<'g', double>();
    program.add_argument("--initWindow")
        .help("Seconds of IMU data averaged to fix the initial gravity direction. 0 (the "
              "default) aligns from the first accelerometer sample alone, which is what the "
              "filter has always done; a few hundred milliseconds averages away the linear "
              "acceleration that a single handheld sample carries into the attitude.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--initVelocityVariance")
        .help("Override the initial velocity variance for host sensitivity studies; negative keeps YAML value.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--initBiasGyrVariance")
        .help("Override the initial gyro-bias variance for host sensitivity studies; negative keeps YAML value.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--rankRecoveryRetention")
        .help("Stage-4 recovery: minimum fraction of features retained after a preceding weak-motion update; "
              "negative disables rank-triggered outlier relaxation.")
        .default_value(-1.0)
        .scan<'g', double>();
    program.add_argument("--seed")
        .help("Seed for the RANSAC and gating random number generator. Every stochastic "
              "part of the pipeline draws from it, so repeating a run with a different "
              "seed measures how much of a result is the method and how much is the draw.")
        .default_value(0)
        .scan<'i', int>();

    // --- Mechanism logs. Each writes one row per image frame. ---
    program.add_argument("--gateLog")
        .help("Per-frame CSV of what the gate decided: box counts, |C_k|, features "
              "rejected by class, and readmissions split into verified and untested. "
              "This is what the regime and mechanism tables are built from.")
        .default_value(std::string(""));
    program.add_argument("--budgetSchedule")
        .help("Open-loop control: file with one maxFeatures value per vision frame, replayed "
              "in order (wrapping) instead of any adaptive budget decision.")
        .default_value(std::string(""));
    program.add_argument("--filterLog")
        .help("Per-frame CSV of estimator conditioning: features reaching the update, "
              "rank and condition number of the sensor block of the output matrix, "
              "innovation covariance, NIS, and the Riccati traces. This is what turns "
              "'the update stayed well conditioned' from an assertion into a measurement.")
        .default_value(std::string(""));
    program.add_argument("--gtDynMask")
        .help("Directory of ground-truth dynamic masks (<timestamp_ns>.png, 255 where "
              "dynamic), as VIODE provides from its segmentation channel. Turns the gate "
              "log into a confusion count: dynamic features admitted, static features "
              "discarded. Without ground truth those two numbers cannot be measured.")
        .default_value(std::string(""));

    // --- Ablation switches. Each removes one mechanism from the adaptive policy. ---
    program.add_argument("--ablateVehicleReject")
        .help("Ablation: drop unconditional vehicle rejection; vehicles go down the "
              "pedestrian path instead.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--ablateStarvationFloor")
        .help("Ablation: drop the starvation floor; pedestrians are blanket-rejected in "
              "every regime however few background features remain.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--ablateClassDiff")
        .help("Ablation: drop class differentiation; every box is handled by one rule.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--ablateBoxPropagation")
        .help("Ablation: freeze boxes between detections instead of carrying them with "
              "the tracker's optical flow.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--vehicleBoxesOnly")
        .help("Ablation: ignore pedestrian boxes entirely.")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("--personBoxesOnly")
        .help("Ablation: ignore vehicle boxes entirely.")
        .default_value(false)
        .implicit_value(true);

    // Read the arguments
    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cout << err.what() << '\n' << program;
        exit(0);
    }

    const std::string configFileName = program.get("config");
    DynamicMaskSource dynMasks(program.get("--dynmask"), program.get<int>("--dynmaskDilate"));
    if (!program.get("--dynmask").empty()) {
        if (!dynMasks.active()) {
            std::cout << "No dynamic masks found in " << program.get("--dynmask")
                      << " -- running WITHOUT masking." << std::endl;
        } else {
            std::cout << "Dynamic masking enabled: " << dynMasks.size() << " masks, dilate "
                      << program.get<int>("--dynmaskDilate") << " px." << std::endl;
        }
    }
    YAML::Node eqvioConfig = YAML::LoadFile(configFileName);
    bool writeStateFlag = configOrDefault(eqvioConfig, "main:writeState", true);
    const double cameraLag = configOrDefault(eqvioConfig, "main:cameraLag", 0.0);

    const bool displayFlag = program.get<bool>("--display");
    const bool simvisFlag = program.get<bool>("--simvis");
    const bool simimuFlag = program.get<bool>("--simimu");
    const bool writeTimeFlag = program.get<bool>("--timing");

    std::unique_ptr<DataServerBase> dataServer;
    std::unique_ptr<DatasetReaderBase> datasetReader =
        createDatasetReader(program.get("--mode"), program.get("dataset"), cameraLag);
    // Defer the estimator's start to the first quasi-static window when the
    // configuration asks for it: initialising while the platform is slowly
    // translating locks the stationary velocity prior against the truth, and
    // at reduced image resolutions the tracker cannot recover from it. The
    // gate must be set before the threaded server starts prefetching.
    if (configOrDefault(eqvioConfig, "main:autoStartGate", false)) {
        datasetReader->enableAutoStartGate(configOrDefault(eqvioConfig, "main:startGateMaxWait", 45.0));
    }
#if EQVIO_SINGLE_THREADED_DATASERVER
    /* ThreadedDataServer runs a reader thread that prefetches and decodes
     * frames ahead of the filter. On a multi-core host that hides the I/O; on
     * the K230, which exposes a single core (docs/17), it only adds contention.
     * Measured on the host, EqVIO averages 21.75 threads and 344 % CPU while
     * the filter itself is one thread -- all of that would be overhead here. */
    dataServer = std::make_unique<SimpleDataServer>(std::move(datasetReader));
#else
    dataServer = std::make_unique<ThreadedDataServer>(std::move(datasetReader), eqvioConfig["sim"]);
#endif

    // Read the camera file if provided
    if (program.is_used("--camera")) {
        dataServer->readCamera(program.get<std::string>("--camera"));
    }

    // Camera-from-IMU rotation, used by the IMU-compensated gate; identity
    // when the dataset carries no extrinsics.
    Eigen::Matrix3d Rci = Eigen::Matrix3d::Identity();
    if (dataServer->cameraExtrinsics())
        Rci = dataServer->cameraExtrinsics()->R.asMatrix();

    // Set up the timer.
    // "detect" is GIFT's CPU corner-detection stage (PointFeatureTracker.cpp) and
    // must not be touched; the neural detector channel gets its own "nn_detect"
    // category so emulated and, later, live accelerator inference is accounted
    // separately from the point detector it replaces.
    loopTimer.initialise({"correction", "features", "propagate", "selective", "preprocessing", "propagation", "total",
        "total vision update", "write output", "image load", "klt", "static ransac", "detect", "nn_detect"});

    const double limitRateSetting = program.get<double>("--limitRate");
    double startTime = program.get<double>("--start");
    double stopTime = program.get<double>("--stop");
    {
        double initialDataTime = dataServer->nextTime();
        startTime = startTime > 0 ? initialDataTime + startTime : startTime;
        stopTime = stopTime > 0 ? initialDataTime + stopTime : stopTime;
    }

    // Read I/O settings

    if (displayFlag && !EQVIO_BUILD_VISUALISATION) {
        std::cout << "Visualisations have been requested, but the necessary module is not set to be built."
                  << std::endl;
    }
    VIOVisualiser visualiser(displayFlag);
    if (displayFlag) {
        visualiser.setGroundTruthTrajectory(dataServer->getTrajectory());
    }

    // Initialise the filter
    VIOFilter::Settings filterSettings(eqvioConfig["eqf"]);
    // A negative value means "leave whatever the configuration says", so the
    // flag can override the config without silently replacing it when unset.
    if (program.get<double>("--initWindow") >= 0.0)
        filterSettings.initialisationWindow = program.get<double>("--initWindow");
    if (program.get<double>("--initVelocityVariance") >= 0.0)
        filterSettings.initialVelocityVariance = program.get<double>("--initVelocityVariance");
    if (program.get<double>("--initBiasGyrVariance") >= 0.0)
        filterSettings.initialBiasOmegaVariance = program.get<double>("--initBiasGyrVariance");
    if (program.get<double>("--rankRecoveryRetention") >= 0.0)
        filterSettings.rankRecoveryRetention = program.get<double>("--rankRecoveryRetention");
    std::cout << "IMU initialisation window: " << filterSettings.initialisationWindow << " s" << std::endl;
    std::cout << "Initial velocity variance: " << filterSettings.initialVelocityVariance
              << ", gyro-bias variance: " << filterSettings.initialBiasOmegaVariance << std::endl;
    if (dataServer->cameraExtrinsics()) {
        std::cout << "Camera extrinsics were provided by the dataset, overriding those in the filter settings."
                  << std::endl;
        filterSettings.cameraOffset = *dataServer->cameraExtrinsics();
    }
    std::cout << "The camera extrinsics (pose of the camera w.r.t. the IMU) are:\n"
              << filterSettings.cameraOffset.asMatrix() << std::endl;
    VIOFilter filter(filterSettings);

    // Initialise the feature tracker
    GIFT::PointFeatureTracker featureTracker = GIFT::PointFeatureTracker(dataServer->camera());
    // GIFT::KeyPointFeatureTracker featureTracker = GIFT::KeyPointFeatureTracker(dataServer->camera);
    featureTracker.settings.configure(eqvioConfig["GIFT"]);
    if (program.get<double>("--featureDistOverride") >= 0.0) {
        featureTracker.settings.featureDist = program.get<double>("--featureDistOverride");
        // The override names a frontend budget, not just a proposal spacing:
        // raising it alone would leave the already-tracked features in place and
        // the run would keep paying for the old budget. Align the pruning with
        // it so a fixed arm really is fixed at that budget. See the N1 block
        // below, where the adaptive policy does the same thing per frame.
        if (featureTracker.settings.trackedFeatureDist < featureTracker.settings.featureDist)
            featureTracker.settings.trackedFeatureDist = featureTracker.settings.featureDist;
    }
    if (program.get<int>("--maxFeaturesOverride") >= 0)
        featureTracker.settings.maxFeatures = program.get<int>("--maxFeaturesOverride");

    // Open-loop control: replay a fixed per-frame cap schedule (line i = cap for
    // vision frame i) regardless of what the estimator reports.
    std::vector<size_t> budgetSchedule;
    if (!program.get("--budgetSchedule").empty()) {
        std::ifstream sched(program.get("--budgetSchedule"));
        for (long cap; sched >> cap;) budgetSchedule.push_back(static_cast<size_t>(std::max(1L, cap)));
        if (budgetSchedule.empty()) {
            std::cerr << "--budgetSchedule: no caps read from " << program.get("--budgetSchedule") << "\n";
            return 1;
        }
        featureTracker.settings.maxFeatures = budgetSchedule.front();
    }

    // ---- Mechanism N1: rank-driven frontend budget ------------------------
    // The budget for frame k+1 follows the motion rank of frame k's vision
    // update. The rank only exists after processVisionData(), so this loop is
    // one frame late by construction -- that is a property of the signal, not
    // an implementation detail, and it is why the policy is smoothed.
    //
    // The dense band is deliberate: rank 0 means the frame carried no parallax
    // (standstill or pure rotation) and adding features cannot create geometry
    // that is not there, while rank 9 means the update already constrains all
    // nine ego-motion directions and needs no help. Both are measured, see
    // notes/research-plan.md and the calibration table in the plan.
    // Two actuators behind one signal. "rank" moves the feature density
    // (featureDist) and therefore the accuracy budget itself; "cadence" leaves
    // featureDist alone and moves maxFeatures, which is what GIFT's skip test
    // reads (features > featureSearchThreshold * maxFeatures), so it only
    // changes how often the whole-image detector runs. Measurement says the
    // second is the one that buys throughput without paying accuracy: detect is
    // ~59 % of the frame, while featureDist is already at the cost floor the
    // configured operating point sits on.
    const std::string adaptiveBudgetMode = program.get("--adaptiveBudget");
    // "starve" is the one that behaves adaptively on the board. The rank-based
    // modes move a signal that saturates there (motionRank sits at 9 for 6 of 7
    // sequences), so they collapse into a constant; the load-bearing observable
    // is instead how many measurements the filter is actually using, which
    // falls from 6-7 to 2-3 exactly when a suppressed detector starves the
    // frontend -- the failure that costs whole trajectories.
    const bool adaptiveBudgetFd = (adaptiveBudgetMode == "rank");
    const bool adaptiveBudgetCd = (adaptiveBudgetMode == "cadence");
    const bool adaptiveBudgetSv = (adaptiveBudgetMode == "starve");
    const bool adaptiveBudget = adaptiveBudgetFd || adaptiveBudgetCd || adaptiveBudgetSv;
    if (!adaptiveBudgetMode.empty() && !adaptiveBudget)
        throw std::runtime_error("unknown --adaptiveBudget '" + adaptiveBudgetMode
                                 + "' (expected 'rank', 'cadence', 'starve' or empty)");

    const double baseFeatureDist = featureTracker.settings.featureDist;
    // Raising featureDist alone only stops the tracker from *adding* features;
    // the ones already tracked stay until they fail, so a short dense burst
    // keeps costing long after the policy went back to sparse. GIFT prunes
    // features closer than trackedFeatureDist every frame, so the sparse state
    // raises that too and actually sheds the excess. The configured value is
    // the floor: the policy may tighten pruning but never loosen what the
    // configuration asked for.
    const double baseTrackedDist = featureTracker.settings.trackedFeatureDist;
    double abDenseDist  = program.get<double>("--adaptiveBudgetDense");
    double abSparseDist = program.get<double>("--adaptiveBudgetSparse");
    if (abDenseDist  < 0.0) abDenseDist  = baseFeatureDist * 0.5;
    if (abSparseDist < 0.0) abSparseDist = baseFeatureDist;
    if (abDenseDist > abSparseDist)
        throw std::runtime_error("--adaptiveBudgetDense must not exceed --adaptiveBudgetSparse");
    const double abRankLow  = program.get<double>("--adaptiveBudgetRankLow");
    const double abRankHigh = program.get<double>("--adaptiveBudgetRankHigh");
    if (abRankHigh <= abRankLow)
        throw std::runtime_error("--adaptiveBudgetRankHigh must exceed --adaptiveBudgetRankLow");
    const double abSmooth = std::min(1.0, std::max(0.0, program.get<double>("--adaptiveBudgetSmooth")));
    const int    abHold   = std::max(0, program.get<int>("--adaptiveBudgetHold"));

    double abRankEma = -1.0;   // unset until the first applied update
    bool   abDense   = false;  // start sparse: the configured, proven budget
    int    abFramesSinceSwitch = abHold;
    int    abSwitches = 0;

    const int abMaxLow  = std::max(1, program.get<int>("--adaptiveBudgetMaxLow"));
    int abMaxHigh = program.get<int>("--adaptiveBudgetMaxHigh");
    if (abMaxHigh < 0) abMaxHigh = static_cast<int>(featureTracker.settings.maxFeatures);
    if (abMaxHigh < abMaxLow) abMaxHigh = abMaxLow;

    const int abMeasFloor = std::max(1, program.get<int>("--adaptiveBudgetMeasFloor"));

    // Self-calibrating floor (internal-calibration principle, part 1): instead
    // of comparing n_meas_used to a hand-picked absolute floor, compare it to
    // a percentile of its own recent history. The same command line then
    // transfers across resolutions/calibrations that yield very different
    // natural feature counts (measured 7.9 vs 37.7 mean n_meas_used between
    // two resolutions on the same sequence) without re-tuning abMeasFloor.
    const bool   abAutoFloor    = program.get<bool>("--adaptiveBudgetAutoFloor");
    const int    abAutoFloorWin = std::max(2, program.get<int>("--adaptiveBudgetAutoFloorWindow"));
    const double abAutoFloorPct = std::min(100.0, std::max(0.0, program.get<double>("--adaptiveBudgetAutoFloorPercentile")));
    std::deque<int> abFloorHistory;

    // Self-damping hysteresis (internal-calibration principle, part 2): grow
    // the effective hold when the policy has itself been flapping recently.
    // Motivated by the V2_03_difficult failure, where a fixed Hold=1 let
    // max_features_applied switch state on 57% of frames (82% of the
    // resulting dwell-times were a single frame) immediately before the
    // filter diverged; Hold=3 fixed that sequence but cost throughput on
    // sequences that were never oscillating in the first place. Growing the
    // hold only when recent switch-rate is actually high avoids paying that
    // cost everywhere.
    const bool   abAutoHold     = program.get<bool>("--adaptiveBudgetAutoHold");
    const bool   abAutoHoldWarmup = program.get<bool>("--adaptiveBudgetAutoHoldWarmup");
    const int    abAutoHoldWin  = std::max(2, program.get<int>("--adaptiveBudgetAutoHoldWindow"));
    const double abAutoHoldGain = program.get<double>("--adaptiveBudgetAutoHoldGain");
    std::deque<int> abSwitchHistory;

    // Self-calibrating ceiling (internal-calibration principle, part 3):
    // breaks the circularity that auto-floor alone cannot -- a floor learned
    // only from n_meas_used history that is itself capped at maxHigh can
    // never discover the scene needs MORE than maxHigh, because the policy
    // has never been permitted to try. Periodically lift the cap to the
    // estimator's own configured (YAML) maxFeatures for a few frames, and
    // set maxHigh from a percentile of what those uncapped probes actually
    // observe.
    const bool   abAutoCeiling      = program.get<bool>("--adaptiveBudgetAutoCeiling");
    const int    abProbePeriod      = std::max(2, program.get<int>("--adaptiveBudgetProbePeriod"));
    const int    abProbeDuration    = std::max(1, program.get<int>("--adaptiveBudgetProbeDuration"));
    const double abProbePercentile  = std::min(100.0, std::max(0.0, program.get<double>("--adaptiveBudgetProbePercentile")));
    const int    abProbeHistorySize = std::max(1, program.get<int>("--adaptiveBudgetProbeHistorySize"));
    const int    abYamlMaxFeatures  = static_cast<int>(featureTracker.settings.maxFeatures);
    int abProbeFrameCounter = 0;   // frames since the last probe cycle started
    bool abProbing = false;
    int  abProbeFramesLeft = 0;
    int  abProbeMaxObserved = 0;
    std::deque<int> abProbeHistory;
    int abMaxHighDynamic = abMaxHigh;   // falls back to the fixed CLI value until calibrated
    const double abLowFraction = program.get<double>("--adaptiveBudgetLowFraction");
    int abMaxLowDynamic = abMaxLow;     // falls back to the fixed CLI value until calibrated

    if (adaptiveBudgetSv) {
        std::cout << "Adaptive budget (N1, starvation guard): maxFeatures " << abMaxLow
                  << " while the filter uses >= " << abMeasFloor
                  << " measurements, " << abMaxHigh << " as soon as it drops below"
                  << std::endl;
        if (abAutoFloor)
            std::cout << "  auto-floor: p" << abAutoFloorPct << " of last " << abAutoFloorWin
                      << " n_meas_used frames (overrides the fixed floor above once warmed up)"
                      << std::endl;
        if (abAutoHold)
            std::cout << "  auto-hold: +" << abAutoHoldGain << " frames per unit recent"
                      << " switch-rate over the last " << abAutoHoldWin << " frames" << std::endl;
        if (abAutoCeiling)
            std::cout << "  auto-ceiling: probe to " << abYamlMaxFeatures << " every "
                      << abProbePeriod << " frames for " << abProbeDuration
                      << " frame(s), maxHigh = p" << abProbePercentile << " of last "
                      << abProbeHistorySize << " probes" << std::endl;
    }

    if (adaptiveBudgetCd) {
        std::cout << "Adaptive budget (N1, cadence): maxFeatures " << abMaxLow
                  << " outside the rank band [" << abRankLow << ", " << abRankHigh
                  << ") and " << abMaxHigh << " inside it; smoothing " << abSmooth
                  << ", hold " << abHold << " frames" << std::endl;
    }

    if (adaptiveBudgetFd) {
        std::cout << "Adaptive budget (N1): dense " << abDenseDist << " px for rank in ["
                  << abRankLow << ", " << abRankHigh << "), sparse " << abSparseDist
                  << " px outside; smoothing " << abSmooth << ", hold " << abHold
                  << " frames" << std::endl;
    }

    // Selective dynamic rejection state. The threshold is GIFT's own epipolar
    // inlier threshold, so the test applied to masked features is exactly the
    // one the static-world RANSAC applies to everything else.
    // Dynamic rejection configuration
    const std::string dynMode = program.get("--dynmaskMode");
    const bool selectiveMode = (dynMode == "selective");
    const bool diffMode = (dynMode == "differentiated");
    const bool blanketMode = (dynMode == "blanket");
    const bool adaptiveMode = (dynMode == "adaptive");
    const bool adaptiveLegacyMode = (dynMode == "adaptive_legacy");
    const bool opdgMode = (dynMode == "opdg");
    const double vehPad = program.get<double>("--dynmaskVehPad");
    const double personPad = program.get<double>("--dynmaskPersonPad");
    const int minSafeClean = program.get<int>("--dynmaskMinClean");
    // The observability guard is deployed per environment: the CLI flag wins
    // when given, otherwise the per-sequence configuration carries the value
    // (0 keeps the guard off).
    int minMotionRank = program.get<int>("--dynmaskMinRank");
    if (!program.is_used("--dynmaskMinRank"))
        minMotionRank = configOrDefault(eqvioConfig, "main:dynmaskMinRank", 0);
    // IMU-compensated verification: the gyro-known inter-frame rotation lets
    // the static-world test run from three clean features instead of eight,
    // which is what keeps the gate verifying inside the starvation regime.
    const bool gateImuVerify = configOrDefault(eqvioConfig, "main:gateImuVerify", false);
    double gateTauImu = configOrDefault(eqvioConfig, "main:gateTauImu", 0.02);
    if (program.get<double>("--gateTauImu") >= 0.0)
        gateTauImu = program.get<double>("--gateTauImu");
    bool gateSoftWeight = configOrDefault(eqvioConfig, "main:gateSoftWeight", true);
    if (program.get<int>("--gateSoftWeight") >= 0)
        gateSoftWeight = program.get<int>("--gateSoftWeight") != 0;
    const bool gateFallbackVerify = program.get<bool>("--dynmaskFallbackVerify");
    const double gateTauSoft = program.get<double>("--gateTauSoft");
    const double softRescueWeight = program.get<double>("--dynmaskSoftRescue");
    const double vehConf = program.get<double>("--dyndetVehConf");
    const double adaptiveRiskAlpha = program.get<double>("--adaptiveRiskAlpha");
    const double adaptiveRiskHigh = program.get<double>("--adaptiveRiskHigh");
    const double adaptiveRiskLow = program.get<double>("--adaptiveRiskLow");
    const int adaptiveRiskRise = program.get<int>("--adaptiveRiskRise");
    const int adaptiveRiskFall = program.get<int>("--adaptiveRiskFall");
    const double adaptiveWeightFloor = program.get<double>("--adaptiveWeightFloor");
    const double adaptiveInfoFloor = program.get<double>("--adaptiveInfoFloor");

    const bool   acovEnabled  = program.get<bool>("--adaptiveCovariance");
    const double acovK1       = program.get<double>("--adaptiveCovarianceK1");
    const double acovK2       = program.get<double>("--adaptiveCovarianceK2");
    const double acovERef     = std::max(1e-6, program.get<double>("--adaptiveCovarianceERef"));
    double acovNTarget        = program.get<double>("--adaptiveCovarianceNTarget");
    if (acovNTarget < 0.0) acovNTarget = static_cast<double>(featureTracker.settings.maxFeatures);
    const double acovAlphaMin = program.get<double>("--adaptiveCovarianceAlphaMin");
    const double acovAlphaMax = program.get<double>("--adaptiveCovarianceAlphaMax");
    const double acovAlphaSlew = program.get<double>("--adaptiveCovarianceAlphaSlew");
    double acovAppliedAlpha = 1.0;
    const bool   acovAutoNTarget = program.get<bool>("--adaptiveCovarianceAutoNTarget");
    const int    acovNTargetWin  = program.get<int>("--adaptiveCovarianceNTargetWindow");
    const double acovNTargetPct  = program.get<double>("--adaptiveCovarianceNTargetPercentile");
    const bool   acovNTargetTracksBudget = program.get<bool>("--adaptiveCovarianceNTargetTracksBudget");
    const double acovNTargetBudgetFrac   = program.get<double>("--adaptiveCovarianceNTargetBudgetFraction");
    std::deque<int> acovNHistory;
    if (acovEnabled)
        std::cout << "Adaptive covariance: alpha_k = clip(1 + " << acovK1 << "*e/(" << acovERef
                  << ") + " << acovK2 << "*(1 - N/"
                  << (acovNTargetTracksBudget ? "budget*" + std::to_string(acovNTargetBudgetFrac)
                                               : (acovAutoNTarget ? "auto" : std::to_string(acovNTarget)))
                  << "), " << acovAlphaMin << ", " << acovAlphaMax
                  << "); e, N from the PREVIOUS frame" << std::endl;

    // Discrete-event timing emulation of the asynchronous accelerator channel
    // (Section 4.2 of the research plan): delay/jitter/drop decide only WHEN a
    // detection becomes visible, and the TTL horizon bounds staleness.
    PropagatedMaskSource::TimingEmulation detTiming;
    detTiming.delayMs = program.get<double>("--dyndetDelayMs");
    detTiming.jitterMs = program.get<double>("--dyndetJitterMs");
    detTiming.dropRate = program.get<double>("--dyndetDropRate");
    detTiming.maxAgeFrames = program.get<int>("--dyndetMaxAgeFrames");
    detTiming.rngSeed = program.get<int>("--seed");
    detTiming.blocking = program.get<bool>("--dyndetBlocking");

    // Freshness-Aware Dynamic Masking: present each propagated box dilated by
    // base + alpha_age*age + beta_motion*|displacement| (plan Section 3.3).
    PropagatedMaskSource::FadmParams fadm;
    fadm.padBase = program.get<double>("--dyndetPadBase");
    fadm.padAge = program.get<double>("--dyndetPadAge");
    fadm.padMotion = program.get<double>("--dyndetPadMotion");
    fadm.padMax = program.get<double>("--dyndetPadMax");

    const std::string detTrackName = program.get("--dyndetTrack");
    const std::string liveDetectorEndpoint = program.get("--dyndetLive");
    PropagatedMaskSource propMasks(program.get("--dyndet"), program.get<int>("--dyndetInterval"),
        program.get<double>("--dyndetGrow"), program.get<double>("--dyndetConf"),
        detTrackName == "roi"          ? PropagatedMaskSource::Track::Roi
        : detTrackName == "redetect"   ? PropagatedMaskSource::Track::Redetect
        : detTrackName == "global"     ? PropagatedMaskSource::Track::Global
        : detTrackName == "internal_only" ? PropagatedMaskSource::Track::InternalOnly
                                          : PropagatedMaskSource::Track::Features,
        vehConf, detTiming, fadm, liveDetectorEndpoint);
    if (!program.get("--dyndet").empty() || !liveDetectorEndpoint.empty()) {
        if (!liveDetectorEndpoint.empty()) {
            std::cout << "Live detector transport: " << liveDetectorEndpoint
                      << " (non-blocking UDP)" << std::endl;
        }
        std::cout << (propMasks.active()
                ? "Propagated masking: " + std::to_string(propMasks.detections()) + " boxes, detector every "
                      + std::to_string(program.get<int>("--dyndetInterval")) + " frames"
                : (liveDetectorEndpoint.empty()
                    ? "No detections found in " + program.get("--dyndet") + " -- running WITHOUT masking"
                    : "Live detector socket ready; awaiting packets"))
                  << std::endl;
        if (detTiming.delayMs > 0.0 || detTiming.jitterMs > 0.0 || detTiming.dropRate > 0.0 ||
            detTiming.maxAgeFrames > 0) {
            std::cout << "Detection channel emulation: delay " << detTiming.delayMs << " ms, jitter +/-"
                      << detTiming.jitterMs << " ms, drop rate " << detTiming.dropRate << ", TTL "
                      << detTiming.maxAgeFrames << " frames, seed " << detTiming.rngSeed << std::endl;
        }
        if (fadm.padBase >= 0.0) {
            std::cout << "FADM: pad base " << fadm.padBase << " px, age rate " << fadm.padAge
                      << " px/ms, motion rate " << fadm.padMotion << " px/px, max " << fadm.padMax << " px"
                      << std::endl;
        }
    }
    const auto& giftSettings = featureTracker.settings;
    const double selThreshArg = program.get<double>("--dynmaskSelThresh");
    const double selectiveThreshold =
        selThreshArg > 0 ? selThreshArg : giftSettings.ransacParams.inlierThreshold;
    const int rngSeed = program.get<int>("--seed");
    std::mt19937 selectiveRng(rngSeed);
    /* The front-end's static-world RANSAC is the pipeline's largest stochastic
     * component -- it culls features on every frame, not just the ones a gate
     * looks at -- and GIFT seeds it with a fixed mt19937(0). Left alone, every
     * "different seed" run is bit-identical and a seed study measures nothing.
     * Both generators are seeded from the same argument so that one number
     * describes the whole run. */
    featureTracker.settings.rng = std::mt19937(rngSeed);

    ablation.vehicleReject = !program.get<bool>("--ablateVehicleReject");
    ablation.starvationFloor = !program.get<bool>("--ablateStarvationFloor");
    ablation.classDiff = !program.get<bool>("--ablateClassDiff");
    ablation.boxPropagation = !program.get<bool>("--ablateBoxPropagation");
    ablation.usePersonBoxes = !program.get<bool>("--vehicleBoxesOnly");
    ablation.useVehicleBoxes = !program.get<bool>("--personBoxesOnly");
    long long nSelectiveDropped = 0, nSelectiveSeen = 0;
    std::cout << "Dynamic masking mode: " << dynMode;
    if (adaptiveMode || adaptiveLegacyMode || opdgMode) {
        std::cout << " (adaptive: vehicle blanket pad=" << vehPad << "px [conf>=" << vehConf
                  << "], person policy when clean>=" << minSafeClean
                  << (opdgMode                ? ", OPDG tiers, w_soft=" + std::to_string(softRescueWeight) + ")"
                      : adaptiveLegacyMode     ? ", legacy)"
                                               : ", temporal risk-information policy)");
    } else if (diffMode) {
        std::cout << " (hard vehicle blanket pad=" << vehPad << "px, selective person thresh=" << selectiveThreshold << ")";
    } else if (selectiveMode) {
        std::cout << " (threshold " << selectiveThreshold << ")";
    } else if (blanketMode) {
        std::cout << " (blanket pad=" << vehPad << "px)";
    }
    std::cout << std::endl;

    // Set up output files

    std::stringstream outputFileNameStream, internalFileNameStream;
    std::time_t t0 = std::time(nullptr);
    if (program.is_used("--output")) {
        outputFileNameStream << program.get<std::string>("--output");
        if (!writeStateFlag) {
            writeStateFlag = true;
            std::cout
                << "User provided an output file, so the output writing flag from the configuration will be ignored."
                << std::endl;
        }
    } else {
        outputFileNameStream << "EQVIO_output_" << std::put_time(std::localtime(&t0), "%F_%T") << "/";
    }
    VIOWriter vioWriter(outputFileNameStream.str());

    if (!program.get("--dyndetTrace").empty()) {
        gateTrace.open(program.get("--dyndetTrace"));
        gateTrace << "stamp,x,y,set,residual,kept\n";
        std::cout << "Writing gate trace to " << program.get("--dyndetTrace") << std::endl;
    }

    /* Ground-truth dynamic masks, where the dataset has them. 255 marks dynamic,
     * so keepMaskAt's inverted result is zero there; dilate is 0 because this is
     * the reference the gate is scored against and must not be grown. */
    DynamicMaskSource gtMasks(program.get("--gtDynMask"), 0);
    if (!program.get("--gtDynMask").empty())
        std::cout << (gtMasks.active() ? "Ground-truth dynamic masks: " + std::to_string(gtMasks.size()) + " frames"
                                       : "No ground-truth masks found in " + program.get("--gtDynMask"))
                  << std::endl;

    std::ofstream gateLog;
    if (!program.get("--gateLog").empty()) {
        gateLog.open(program.get("--gateLog"));
        gateLog << "frame,stamp,regime,n_tracked,n_veh_boxes,n_person_boxes,n_veh_rejected,"
                   "n_box_rejected,n_ped_rejected,n_clean,n_suspect,n_tested,"
                   "n_readmit_verified,n_readmit_unverified,n_model_failures,n_out,"
                   "n_risk_removed,n_risk_weighted,n_temporal_moving,risk_mean,"
                   "information_mean,weight_mean,gt_dyn_kept,gt_dyn_rejected,"
                   "gt_static_kept,gt_static_rejected,"
                   // Detection-channel telemetry (freshness, source accounting).
                   "det_status,det_age_frames,det_age_ms,det_latency_ms,"
                   "det_n_internal,det_n_global,det_n_frozen,"
                   "det_packets_dropped_total,det_ttl_expired_total,det_pad_mean_px\n";
    }

    std::ofstream filterLog;
    if (!program.get("--filterLog").empty()) {
        filterLog.open(program.get("--filterLog"));
        filterLog << "frame,stamp,integrated,update_applied,cholesky_ok,n_meas_in,n_meas_used,"
                     "n_outliers,n_landmarks,meas_dim,sensor_info_rank,sensor_info_cond,"
                     "sensor_info_min,sensor_info_max,motion_info_rank,motion_info_min,"
                     "innovation_rms,innovation_cov_trace,nis,"
                     "trace_sigma_att,trace_sigma_pos,trace_sigma_vel,trace_sigma_bias,trace_sigma_sensor,"
                     "rank_recovery,feature_dist_applied,max_features_applied,rank_ema\n";
    }

    /* One row naming every setting this run used. The sweeps and ablations are
     * dozens of runs whose only difference is an argument, and an output
     * directory name is not a record of what produced it. */
    {
        std::ofstream cfgLog(outDirOf(outputFileNameStream.str()) + "run_config.csv");
        cfgLog << "key,value\n"
               << "dataset," << program.get("dataset") << "\n"
               << "config," << configFileName << "\n"
               << "mode," << dynMode << "\n"
               << "policy_version," << (adaptiveMode ? "adaptive_v2" : (adaptiveLegacyMode ? "adaptive_legacy" : dynMode)) << "\n"
               << "seed," << rngSeed << "\n"
               << "detector_csv," << program.get("--dyndet") << "\n"
               << "detector_live_endpoint," << liveDetectorEndpoint << "\n"
               << "detector_interval," << program.get<int>("--dyndetInterval") << "\n"
               << "detector_track," << detTrackName << "\n"
               << "detector_grow," << program.get<double>("--dyndetGrow") << "\n"
               << "detector_delay_ms," << detTiming.delayMs << "\n"
               << "detector_jitter_ms," << detTiming.jitterMs << "\n"
               << "detector_drop_rate," << detTiming.dropRate << "\n"
               << "detector_max_age_frames," << detTiming.maxAgeFrames << "\n"
               << "detector_blocking," << detTiming.blocking << "\n"
               << "soft_rescue_weight," << softRescueWeight << "\n"
               << "fadm_pad_base," << fadm.padBase << "\n"
               << "fadm_pad_age," << fadm.padAge << "\n"
               << "fadm_pad_motion," << fadm.padMotion << "\n"
               << "fadm_pad_max," << fadm.padMax << "\n"
               << "feature_dist," << featureTracker.settings.featureDist << "\n"
               << "max_features," << featureTracker.settings.maxFeatures << "\n"
               << "adaptive_budget," << adaptiveBudgetMode << "\n"
               << "adaptive_dense_dist," << abDenseDist << "\n"
               << "adaptive_sparse_dist," << abSparseDist << "\n"
               << "adaptive_rank_low," << abRankLow << "\n"
               << "adaptive_rank_high," << abRankHigh << "\n"
               << "adaptive_smooth," << abSmooth << "\n"
               << "adaptive_hold," << abHold << "\n"
               << "adaptive_max_low," << abMaxLow << "\n"
               << "adaptive_max_high," << abMaxHigh << "\n"
               << "adaptive_meas_floor," << abMeasFloor << "\n"
               << "conf_person," << program.get<double>("--dyndetConf") << "\n"
               << "conf_vehicle," << vehConf << "\n"
               << "pad_vehicle," << vehPad << "\n"
               << "pad_person," << personPad << "\n"
               << "n_safe," << minSafeClean << "\n"
               << "selective_threshold," << selectiveThreshold << "\n"
               << "gate_imu_verify," << gateImuVerify << "\n"
               << "gate_tau_imu," << gateTauImu << "\n"
               << "gate_soft_weight," << gateSoftWeight << "\n"
               << "min_motion_rank," << minMotionRank << "\n"
               << "adaptive_risk_alpha," << adaptiveRiskAlpha << "\n"
               << "adaptive_risk_high," << adaptiveRiskHigh << "\n"
               << "adaptive_risk_low," << adaptiveRiskLow << "\n"
               << "adaptive_risk_rise," << adaptiveRiskRise << "\n"
               << "adaptive_risk_fall," << adaptiveRiskFall << "\n"
               << "adaptive_weight_floor," << adaptiveWeightFloor << "\n"
               << "adaptive_information_floor," << adaptiveInfoFloor << "\n"
               << "initialisation_window," << filterSettings.initialisationWindow << "\n"
               << "initial_velocity_variance," << filterSettings.initialVelocityVariance << "\n"
               << "initial_bias_gyr_variance," << filterSettings.initialBiasOmegaVariance << "\n"
               << "rank_recovery_retention," << filterSettings.rankRecoveryRetention << "\n"
               << "ablate_vehicle_reject," << (!ablation.vehicleReject) << "\n"
               << "ablate_starvation_floor," << (!ablation.starvationFloor) << "\n"
               << "ablate_class_diff," << (!ablation.classDiff) << "\n"
               << "ablate_box_propagation," << (!ablation.boxPropagation) << "\n"
               << "vehicle_boxes_only," << (!ablation.usePersonBoxes) << "\n"
               << "person_boxes_only," << (!ablation.useVehicleBoxes) << "\n";
    }

    int imuDataCounter = 0, visionDataCounter = 0;

    // Memory milestones. The first is taken here, before any frame is read, so
    // it charges the shared vision library and the process image but nothing of
    // the per-frame working set. See docs/36.
    const std::string memoryDir = outDirOf(outputFileNameStream.str());
    std::ofstream memoryFile(memoryDir + "memory.csv");
    memoryFile << "frame, rss_kib, peak_kib\n";
    const long rssAtStart = residentKiB().first;
    memoryFile << "0, " << rssAtStart << ", " << residentKiB().second << "\n";

    const std::chrono::steady_clock::time_point loopStartTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point rateLimitTimer = std::chrono::steady_clock::now();
    const bool pacedReplay = program.get<bool>("--pacedReplay");
    if (pacedReplay && limitRateSetting > 0) {
        throw std::invalid_argument("--pacedReplay requires --limitRate 0");
    }
    std::ofstream replayFile;
    if (pacedReplay) {
        replayFile.open(memoryDir + "realtime.csv");
        replayFile << "frame,stamp,arrival_s,start_s,finish_s\n" << std::setprecision(17);
    }
    double replayFirstStamp = -1;
    std::chrono::steady_clock::time_point replayEpoch;

    while (true) {
        MeasurementType measType = dataServer->nextMeasurementType();

        if (measType == MeasurementType::None) {
            // End of data
            break;
        }
        const double nextStamp = dataServer->nextTime();
        double replayArrival = 0, replayStart = 0;
        if (pacedReplay && (startTime <= 0 || nextStamp >= startTime)) {
            if (replayFirstStamp < 0) {
                replayFirstStamp = nextStamp;
                replayEpoch = std::chrono::steady_clock::now();
                std::ofstream epochFile(memoryDir + "replay_epoch.txt");
                epochFile << std::setprecision(17)
                          << std::chrono::duration<double>(replayEpoch.time_since_epoch()).count() << '\n';
            }
            replayArrival = nextStamp - replayFirstStamp;
            std::this_thread::sleep_until(replayEpoch + std::chrono::duration<double>(replayArrival));
            replayStart = std::chrono::duration<double>(std::chrono::steady_clock::now() - replayEpoch).count();
        }

        if (measType == MeasurementType::Image) {
            loopTimer.startLoop();

            StampedImage imageData;
            VisionMeasurement measData;
            // Gate-vs-ground-truth confusion counts for this frame; all zero
            // unless --gtDynMask supplied a reference to score against.
            int gtDynKept = 0, gtDynRejected = 0, gtStaticKept = 0, gtStaticRejected = 0;

            loopTimer.startTiming("total");

            if (simvisFlag) {
                measData = dataServer->getSimVision();
                if (startTime > 0 && measData.stamp < startTime) {
                    continue;
                }
            } else {
                loopTimer.startTiming("image load");
                imageData = dataServer->getImage();
                loopTimer.endTiming("image load");
                if (startTime > 0 && imageData.stamp < startTime) {
                    continue;
                }
                loopTimer.startTiming("features");
                cv::Mat keepMask;
                if (dynMasks.active()) {
                    keepMask = dynMasks.keepMaskAt(imageData.stamp);
                    // In blanket mode GIFT applies the mask itself, to both the
                    // culling of existing tracks and the detection of new ones.
                    // In selective mode it must NOT, or the features would be
                    // gone before their geometry could be tested; the mask is
                    // still used to steer detection away from the object.
                    featureTracker.setMask(selectiveMode ? cv::Mat() : keepMask);
                }
                const VisionMeasurement& featurePrediction =
                    filter.getFeaturePredictions(dataServer->camera(), imageData.stamp);
                featureTracker.processImage(imageData.image, featurePrediction.ocvCoordinates());
                std::vector<GIFT::Feature> outFeatures = featureTracker.outputFeatures();

                gateStats.reset();
                gateNoiseWeights.clear();
                gateStats.nTracked = static_cast<int>(outFeatures.size());
                // Kept only to score the gate against ground truth; without GT
                // masks the copy is skipped, since it is a per-frame allocation
                // on a pipeline whose whole budget is 30 ms.
                std::vector<GIFT::Feature> featuresBeforeGate;
                if (gtMasks.active())
                    featuresBeforeGate = outFeatures;

                if (propMasks.active()) {
                    // Boxes are advanced with THIS frame's tracked features, so
                    // this has to come after processImage. Detection-driven
                    // masking is therefore selective-only -- the mode that
                    // survives an approximate mask anyway (docs/34 section 6).
                    // No mask image is rasterised: selective rejection only needs
                    // to know whether a feature falls inside a box.
                    loopTimer.startTiming("propagate");
                    propMasks.advance(imageData.stamp, outFeatures, imageData.image);
                    loopTimer.endTiming("propagate");
                    // The boxes belong to the frame, not to any one feature, so
                    // they are written once here with set = 2 and the rectangle
                    // in the x,y,residual,kept columns. Ugly, but it keeps the
                    // trace a single flat CSV that a plotting script can read
                    // without a second format.
                    if (gateTrace.is_open()) {
                        gateTraceStamp = imageData.stamp;
                        for (const auto& b : propMasks.currentBoxes())
                            gateTrace << std::setprecision(17) << gateTraceStamp
                                      << std::setprecision(7) << "," << b.x << "," << b.y
                                      << ",2," << b.width << "," << b.height << "\n";
                    }
                    loopTimer.startTiming("selective");
                    const size_t before = outFeatures.size();
                    if (adaptiveMode) {
                        // The proposed adaptive policy combines temporal risk,
                        // a per-feature information proxy, and the measured
                        // post-update motion rank as its observability guard.
                        const Eigen::Matrix3d dRc =
                            Rci * imuMotion.captureFrameDelta().transpose() * Rci.transpose();
                        outFeatures = adaptiveDynamicRejection(outFeatures,
                            propMasks.currentVehicleBoxes(), propMasks.currentPersonBoxes(),
                            giftSettings.ransacParams, selectiveRng, selectiveThreshold, vehPad,
                            personPad, static_cast<size_t>(minSafeClean), minMotionRank, dRc,
                            gateTauImu, gateImuVerify, gateSoftWeight, gateFallbackVerify,
                            gateTauSoft, adaptiveRiskAlpha, adaptiveRiskHigh, adaptiveRiskLow,
                            adaptiveRiskRise, adaptiveRiskFall, adaptiveWeightFloor, adaptiveInfoFloor);
                    } else if (adaptiveLegacyMode || opdgMode) {
                        // Gyro-known rotation, expressed in the camera frame,
                        // turns the static-world test linear in the
                        // translation direction -- verification survives the
                        // starvation regime. The 'opdg' mode is the same tiered
                        // gate with the geometric tier always on (it is
                        // definitional to OPDG) and the soft-rescue weight
                        // exposed, so B8 is self-describing in run_config.csv.
                        const Eigen::Matrix3d dRc =
                            Rci * imuMotion.captureFrameDelta().transpose() * Rci.transpose();
                        outFeatures = adaptiveLegacyDynamicRejection(outFeatures,
                            propMasks.currentVehicleBoxes(), propMasks.currentPersonBoxes(),
                            giftSettings.ransacParams, selectiveRng, selectiveThreshold, vehPad,
                            personPad, static_cast<size_t>(minSafeClean), minMotionRank, dRc,
                            gateTauImu, gateImuVerify || opdgMode, gateSoftWeight, gateFallbackVerify,
                            gateTauSoft, softRescueWeight);
                    } else if (diffMode) {
                        outFeatures = differentiatedDynamicRejection(outFeatures,
                            propMasks.currentVehicleBoxes(), propMasks.currentPersonBoxes(),
                            giftSettings.ransacParams, selectiveRng, selectiveThreshold, vehPad);
                    } else if (blanketMode) {
                        outFeatures = blanketDynamicRejection(outFeatures,
                            propMasks.currentBoxes(), vehPad, minMotionRank);
                    } else { // selective
                        outFeatures = selectiveDynamicRejection(outFeatures, propMasks.currentBoxes(),
                            giftSettings.ransacParams, selectiveRng, selectiveThreshold);
                    }
                    nSelectiveDropped += before - outFeatures.size();
                    nSelectiveSeen += before;
                    loopTimer.endTiming("selective");
                } else if (selectiveMode && !keepMask.empty()) {
                    const size_t before = outFeatures.size();
                    outFeatures = selectiveDynamicRejection(
                        outFeatures, keepMask, giftSettings.ransacParams, selectiveRng, selectiveThreshold);
                    nSelectiveDropped += before - outFeatures.size();
                    nSelectiveSeen += before;
                }
                gateStats.nOut = static_cast<int>(outFeatures.size());

                /* Score the gate against ground truth, where there is any. A
                 * feature is dynamic if it sits on a pixel the simulator marked
                 * as a moving object; the four counts are then the confusion
                 * matrix of the gate's keep/reject decision against that label.
                 * gt_dyn_kept is the contamination the gate let through and
                 * gt_static_rejected is the good measurement it threw away --
                 * the two error modes the policy trades against each other, and
                 * neither is observable from a trajectory alone. */
                if (gtMasks.active()) {
                    const cv::Mat gtKeep = gtMasks.keepMaskAt(imageData.stamp);
                    if (!gtKeep.empty()) {
                        std::set<int> keptIds;
                        for (const auto& f : outFeatures)
                            keptIds.insert(f.idNumber);
                        for (const auto& f : featuresBeforeGate) {
                            const int x = cvRound(f.camCoordinates.x), y = cvRound(f.camCoordinates.y);
                            if (x < 0 || y < 0 || x >= gtKeep.cols || y >= gtKeep.rows)
                                continue;
                            const bool dynamic = gtKeep.at<uchar>(y, x) == 0;
                            const bool kept = keptIds.count(f.idNumber) > 0;
                            if (dynamic)
                                kept ? ++gtDynKept : ++gtDynRejected;
                            else
                                kept ? ++gtStaticKept : ++gtStaticRejected;
                        }
                    }
                }

                // Global, frame-level adaptive measurement covariance:
                // R_k = alpha_k * R_0, alpha_k a function of the PREVIOUS
                // frame's innovation RMS (e) and n_meas_used (N) -- both are
                // read from the global eqfDiagnostics left over from that
                // frame's update, since the current frame's own residual
                // will not exist until AFTER this measurement is fused.
                // Applied as a uniform per-feature weight multiplied on top
                // of whatever the dynamic-object gate already set (empty on
                // a non-gated run, in which case this is the only source of
                // noiseWeights and the fast/empty-map path in
                // VIOFilter.cpp's correction step is skipped for this run).
                if (acovEnabled && eqfDiagnostics.applied) {
                    // Self-calibrated N_target: judged against a percentile of this
                    // run's OWN recent n_meas_used (accumulated before this frame is
                    // added to it), not the blanket YAML maxFeatures. A scene that
                    // structurally never reaches that ceiling (e.g. a tight
                    // featureDist on a narrow image) otherwise has (1 - N/N_target)
                    // pinned near its max every frame -- indistinguishable from
                    // genuine corruption -- which chronically inflates alpha_k and
                    // starves the filter's corrections into a dead-reckoning drift.
                    // When a budget mechanism is also adapting maxFeatures, a
                    // low n_meas_used during its own aggressive/cut phase is
                    // expected, not corruption -- judge N against the cap
                    // that is actually active THIS frame (adaptiveBudget's
                    // decision block, further below, already set it for
                    // this frame before we get here on the next iteration;
                    // read here it is still the cap this frame is running
                    // under) instead of a scene-wide history statistic that
                    // cannot tell a budget cut from a genuine tracking loss.
                    double effectiveNTarget = acovNTarget;
                    if (acovNTargetTracksBudget)
                        effectiveNTarget = std::max(1.0,
                            acovNTargetBudgetFrac * static_cast<double>(featureTracker.settings.maxFeatures));
                    else if (acovAutoNTarget && static_cast<int>(acovNHistory.size()) >= acovNTargetWin / 2)
                        effectiveNTarget = std::max(1.0, percentileOf(acovNHistory, acovNTargetPct));

                    const double eTerm = eqfDiagnostics.innovationRms / acovERef;
                    const double nTerm = 1.0 - static_cast<double>(eqfDiagnostics.nMeasUsed) / effectiveNTarget;
                    double alphaTarget = 1.0 + acovK1 * eTerm + acovK2 * nTerm;
                    alphaTarget = std::min(acovAlphaMax, std::max(acovAlphaMin, alphaTarget));
                    double alpha = alphaTarget;
                    if (acovAlphaSlew >= 0.0) {
                        const double step = std::min(acovAlphaSlew, std::max(-acovAlphaSlew, alphaTarget - acovAppliedAlpha));
                        acovAppliedAlpha += step;
                        alpha = acovAppliedAlpha;
                    }
                    const double w = 1.0 / std::sqrt(alpha); // R *= alpha  <=>  weight = 1/sqrt(alpha)
                    for (const auto& f : outFeatures) {
                        const auto it = gateNoiseWeights.find(f.idNumber);
                        gateNoiseWeights[f.idNumber] = (it != gateNoiseWeights.end()) ? it->second * w : w;
                    }

                    if (acovAutoNTarget) {
                        acovNHistory.push_back(static_cast<int>(eqfDiagnostics.nMeasUsed));
                        if (static_cast<int>(acovNHistory.size()) > acovNTargetWin) acovNHistory.pop_front();
                    }
                }

                measData = convertGIFTFeatures(outFeatures, imageData.stamp);
                measData.cameraPtr = dataServer->camera();
                measData.noiseWeights = gateNoiseWeights;
                loopTimer.endTiming("features");
            }

            loopTimer.startTiming("total vision update");
            filter.processVisionData(measData);
            loopTimer.endTiming("total vision update");

            // Mechanism N1: fold this frame's motion rank into the smoothed
            // signal the budget follows. Done before the log row below so that
            // row carries the value the decision afterwards will act on.
            if (adaptiveBudget && eqfDiagnostics.applied) {
                const double rankNow = static_cast<double>(eqfDiagnostics.motionRank);
                abRankEma = (abRankEma < 0.0)
                    ? rankNow
                    : (1.0 - abSmooth) * abRankEma + abSmooth * rankNow;
            }

            loopTimer.endTiming("total");
            ++visionDataCounter;

            if (gateLog.is_open()) {
                gateLog << visionDataCounter << "," << std::setprecision(17) << measData.stamp
                        << std::setprecision(7) << "," << gateRegimeName(gateStats.regime) << ","
                        << gateStats.nTracked << "," << gateStats.nVehBoxes << ","
                        << gateStats.nPersonBoxes << "," << gateStats.nVehRejected << ","
                        << gateStats.nBoxRejected << "," << gateStats.nPedRejected << ","
                        << gateStats.nClean << "," << gateStats.nSuspect << "," << gateStats.nTested
                        << "," << gateStats.nReadmittedVerified << "," << gateStats.nReadmittedUnverified
                        << "," << gateStats.nModelFailures
                        << "," << gateStats.nOut << "," << gateStats.nRiskRemoved << ","
                        << gateStats.nRiskWeighted << "," << gateStats.nTemporalMoving << ","
                        << gateStats.riskMean << "," << gateStats.informationMean << ","
                        << gateStats.weightMean << "," << gtDynKept << "," << gtDynRejected << ","
                        << gtStaticKept << "," << gtStaticRejected << ","
                        << propMasks.statusName() << "," << propMasks.ageFrames() << ","
                        << propMasks.ageMs() << "," << propMasks.latencyMs() << ","
                        << propMasks.srcInternal() << "," << propMasks.srcGlobal() << ","
                        << propMasks.srcFrozen() << "," << propMasks.packetsDropped() << ","
                        << propMasks.ttlExpiries() << "," << propMasks.padMean() << "\n";
            }
            if (filterLog.is_open()) {
                const EqfDiagnostics& d = eqfDiagnostics;
                filterLog << visionDataCounter << "," << std::setprecision(17) << measData.stamp
                          << std::setprecision(9) << "," << d.integrated << "," << d.applied << ","
                          << d.choleskyOk << "," << d.nMeasIn << "," << d.nMeasUsed << "," << d.nOutliers
                          << "," << d.nLandmarks << "," << d.measDim << "," << d.sensorRank << ","
                          << d.sensorCond << "," << d.sensorSigmaMin << "," << d.sensorSigmaMax << ","
                          << d.motionRank << "," << d.motionInfoMin << ","
                          << d.innovationRms << "," << d.innovationCovTrace << "," << d.nis << ","
                          << d.traceSigmaAtt << "," << d.traceSigmaPos << "," << d.traceSigmaVel << ","
                     << d.traceSigmaBias << "," << d.traceSigmaSensor << "," << d.rankRecovery << ","
                     << featureTracker.settings.featureDist << ","
                     << featureTracker.settings.maxFeatures << ","
                     << (abRankEma < 0.0 ? 0.0 : abRankEma) << "\n";
            }

            if (!budgetSchedule.empty())
                featureTracker.settings.maxFeatures = budgetSchedule[visionDataCounter % budgetSchedule.size()];

            // Mechanism N1: pick the spacing for the NEXT frame. Deliberately
            // after the log row, so feature_dist_applied there is the spacing
            // this frame actually ran with.
            if (adaptiveBudgetSv && abAutoCeiling && abProbeFramesLeft > 0) {
                // Mid-probe: this frame's n_meas_used was measured with the
                // cap lifted, so it is a clean (uncapped) reading of what the
                // scene can actually support -- not something the ordinary
                // floor/hold decision below is allowed to see or act on.
                if (eqfDiagnostics.applied)
                    abProbeMaxObserved = std::max(abProbeMaxObserved, static_cast<int>(eqfDiagnostics.nMeasUsed));
                --abProbeFramesLeft;
                if (abProbeFramesLeft == 0) {
                    abProbeHistory.push_back(abProbeMaxObserved);
                    if (static_cast<int>(abProbeHistory.size()) > abProbeHistorySize) abProbeHistory.pop_front();
                    if (abProbeHistory.size() >= 2) {
                        abMaxHighDynamic = std::max(abMaxLow, static_cast<int>(std::lround(
                            percentileOf(abProbeHistory, abProbePercentile))));
                        // The occasional high->low swing is itself destabilising
                        // if it is too large, regardless of how rarely it fires
                        // (a single 36->3 drop was enough to diverge
                        // MH_02_easy even though --adaptiveBudgetAutoHold's
                        // switch-rate view never saw it as flapping). Scale
                        // the low end with the calibrated ceiling so the swing
                        // stays proportionally the same size the fixed
                        // maxLow=3/maxHigh=7 pairing was tuned around.
                        if (abLowFraction >= 0.0)
                            abMaxLowDynamic = std::max(1, static_cast<int>(std::lround(
                                abLowFraction * abMaxHighDynamic)));
                    }
                    featureTracker.settings.maxFeatures = static_cast<size_t>(abMaxHighDynamic);
                } else {
                    featureTracker.settings.maxFeatures = static_cast<size_t>(abYamlMaxFeatures);
                }
            } else if (adaptiveBudgetSv) {
                const int ceilingNow = abAutoCeiling ? abMaxHighDynamic : abMaxHigh;
                const int floorLowNow = (abAutoCeiling && abLowFraction >= 0.0) ? abMaxLowDynamic : abMaxLow;

                // Self-calibrating floor: judge this frame's n_meas_used
                // against a percentile of its OWN recent history (accumulated
                // before this frame is added to it), not a hand-picked
                // absolute count. Falls back to the fixed --adaptiveBudgetMeasFloor
                // until the window has enough samples to be meaningful.
                int effectiveFloor = abMeasFloor;
                if (abAutoFloor && static_cast<int>(abFloorHistory.size()) >= abAutoFloorWin / 2)
                    effectiveFloor = std::max(1, static_cast<int>(std::lround(
                        percentileOf(abFloorHistory, abAutoFloorPct))));

                // Asymmetric on purpose. Falling below the floor re-enables
                // detection immediately: the frame after a collapse is the one
                // where a late reaction is unrecoverable. Climbing back to the
                // aggressive setting waits for `hold` healthy frames, so a
                // single lucky frame during a recovery does not re-arm it.
                const bool starving = eqfDiagnostics.applied &&
                    static_cast<int>(eqfDiagnostics.nMeasUsed) < effectiveFloor;

                // Self-damping hold: widen the effective hold when the policy
                // has itself been flapping recently (high switch-rate over
                // abSwitchHistory), so a sequence caught oscillating gets more
                // hysteresis exactly when it needs it, while a sequence that
                // never flaps keeps the short, responsive base hold.
                int effectiveHold = abHold;
                if (abAutoHold && !abSwitchHistory.empty() &&
                    (!abAutoHoldWarmup || static_cast<int>(abSwitchHistory.size()) >= abAutoHoldWin / 2)) {
                    const double switchRate = std::accumulate(abSwitchHistory.begin(),
                        abSwitchHistory.end(), 0) / static_cast<double>(abSwitchHistory.size());
                    effectiveHold = abHold + static_cast<int>(std::lround(abAutoHoldGain * switchRate));
                }

                const bool wasManagedCap = featureTracker.settings.maxFeatures == static_cast<size_t>(floorLowNow)
                    || featureTracker.settings.maxFeatures == static_cast<size_t>(ceilingNow);
                bool switched = false;
                if (starving) {
                    if (featureTracker.settings.maxFeatures != static_cast<size_t>(ceilingNow))
                        switched = true;
                    featureTracker.settings.maxFeatures = static_cast<size_t>(ceilingNow);
                    abFramesSinceSwitch = 0;
                } else if (featureTracker.settings.maxFeatures != static_cast<size_t>(floorLowNow)) {
                    if (++abFramesSinceSwitch >= effectiveHold) {
                        featureTracker.settings.maxFeatures = static_cast<size_t>(floorLowNow);
                        switched = true;
                    }
                } else {
                    ++abFramesSinceSwitch;
                }
                if (switched) ++abSwitches;

                if (eqfDiagnostics.applied) {
                    abFloorHistory.push_back(static_cast<int>(eqfDiagnostics.nMeasUsed));
                    if (static_cast<int>(abFloorHistory.size()) > abAutoFloorWin) abFloorHistory.pop_front();
                }
                if (abAutoHold) {
                    abSwitchHistory.push_back(switched && (!abAutoHoldWarmup || wasManagedCap) ? 1 : 0);
                    if (static_cast<int>(abSwitchHistory.size()) > abAutoHoldWin) abSwitchHistory.pop_front();
                }

                // Decide whether the NEXT frame starts a new ceiling probe.
                // This deliberately overrides whatever budget was just picked
                // above -- the probe takes priority for that one frame.
                if (abAutoCeiling) {
                    if (++abProbeFrameCounter >= abProbePeriod) {
                        abProbeFrameCounter = 0;
                        abProbeFramesLeft = abProbeDuration;
                        abProbeMaxObserved = 0;
                        featureTracker.settings.maxFeatures = static_cast<size_t>(abYamlMaxFeatures);
                    }
                }
            } else if (adaptiveBudget) {
                const bool wantDense = (abRankEma >= abRankLow && abRankEma < abRankHigh);
                if (wantDense != abDense && abFramesSinceSwitch >= abHold) {
                    abDense = wantDense;
                    if (adaptiveBudgetCd) {
                        // In the band -> allow detection to replenish; outside it
                        // (rank 0 or rank 9) -> suppress it.
                        featureTracker.settings.maxFeatures =
                            static_cast<size_t>(abDense ? abMaxHigh : abMaxLow);
                    } else {
                    featureTracker.settings.featureDist = abDense ? abDenseDist : abSparseDist;
                    // Prune at whichever spacing is in force, exactly as the
                    // fixed arms do, so an adaptive arm and a fixed arm at the
                    // same budget are the same run. Using baseTrackedDist in
                    // dense mode instead would leave the adaptive arm tracking
                    // more features than the fixed arm it is compared against.
                    featureTracker.settings.trackedFeatureDist =
                        std::max(baseTrackedDist, abDense ? abDenseDist : abSparseDist);
                    }
                    abFramesSinceSwitch = 0;
                    ++abSwitches;
                } else {
                    ++abFramesSinceSwitch;
                }
            }

            if (displayFlag && !simvisFlag) {
                visualiser.displayFeatureImage(featureTracker.outputFeatures(), imageData.image);
            }

            // Output filter data
            loopTimer.startTiming("write output");
            VIOState estimatedState = filter.stateEstimate();
            if (writeStateFlag) {
                vioWriter.writeStates(filter.getTime(), estimatedState);
                vioWriter.writeFeatures(measData);
            }

            loopTimer.endTiming("write output");
            if (pacedReplay) {
                const double replayFinish =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - replayEpoch).count();
                replayFile << visionDataCounter << ',' << nextStamp << ',' << replayArrival << ','
                           << replayStart << ',' << replayFinish << '\n';
            }
            if (writeTimeFlag) {
                vioWriter.writeTiming(loopTimer.getLoopTimingData());
            }

            // The first frame and then every 100th: enough to show whether the
            // footprint is flat or growing, at a cost of one /proc read per
            // 100 frames.
            if (visionDataCounter == 1 || visionDataCounter % 100 == 0) {
                const auto [rss, peak] = residentKiB();
                memoryFile << visionDataCounter << ", " << rss << ", " << peak << "\n";
            }

            // Optionally visualise the filter data
            if (displayFlag) {
                visualiser.updateMapDisplay(estimatedState, filter.getTime());
            }

            // Limit the loop rate
            if (limitRateSetting > 0) {
                std::this_thread::sleep_until(rateLimitTimer + std::chrono::duration<double>(1.0 / limitRateSetting));
                rateLimitTimer = std::chrono::steady_clock::now();
            }

        } else if (measType == MeasurementType::IMU) {
            IMUVelocity imuData;
            if (simimuFlag) {
                imuData = dataServer->getSimIMU();
            } else {
                imuData = dataServer->getIMU();
            }
            assert(!imuData.acc.hasNaN());
            assert(!imuData.gyr.hasNaN());
            assert(imuData.stamp > 0);

            if (startTime > 0 && imuData.stamp < startTime) {
                continue;
            }

            imuMotion.feed(imuData);
            filter.processIMUData(imuData);
            ++imuDataCounter;
        }

        if (stopTime > 0 && filter.getTime() > stopTime) {
            break;
        }
    }

    const auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loopStartTime);
    std::cout << "Processed " << imuDataCounter << " IMU and " << visionDataCounter << " vision measurements.\n"
              << "Time taken: " << elapsedTime.count() * 1e-3 << " seconds." << std::endl;
    {
        const auto [rss, peak] = residentKiB();
        memoryFile << visionDataCounter << ", " << rss << ", " << peak << "\n";
        memoryFile.flush();
        std::cout << "Memory: " << rssAtStart << " KiB before the first frame, " << rss << " KiB at the end, " << peak
                  << " KiB peak." << std::endl;
    }
    if (propMasks.active()) {
        std::cout << "Propagated masks: " << propMasks.detected() << " detector frames, "
                  << propMasks.propagated() << " propagated, " << propMasks.frozen() << " frozen."
                  << std::endl;
        if (detTiming.delayMs > 0.0 || detTiming.jitterMs > 0.0 || detTiming.dropRate > 0.0 ||
            detTiming.maxAgeFrames > 0) {
            std::cout << "Detection channel: " << propMasks.packetsArrived() << " packets arrived, "
                      << propMasks.packetsDropped() << " dropped, " << propMasks.ttlExpiries()
                      << " TTL expiries." << std::endl;
        }
        if (nSelectiveSeen)
            std::cout << (adaptiveMode ? "Adaptive" : (diffMode ? "Differentiated" : (blanketMode ? "Blanket" : "Selective")))
                      << " rejection dropped " << nSelectiveDropped << " of " << nSelectiveSeen
                      << " feature observations (" << (100.0 * nSelectiveDropped / nSelectiveSeen) << " %)."
                      << std::endl;
    }
    if (dynMasks.active()) {
        std::cout << "Dynamic masks applied to " << dynMasks.applied() << " frames, " << dynMasks.missed()
                  << " with no matching mask." << std::endl;
        if (selectiveMode && nSelectiveSeen)
            std::cout << "Selective rejection dropped " << nSelectiveDropped << " of " << nSelectiveSeen
                      << " feature observations (" << (100.0 * nSelectiveDropped / nSelectiveSeen) << " %)."
                      << std::endl;
    }

    if (adaptiveBudget) {
        std::cout << "Adaptive budget (N1): " << abSwitches << " switches between "
                  << abDenseDist << " px (dense) and " << abSparseDist << " px (sparse); "
                  << "final rank EMA " << (abRankEma < 0.0 ? 0.0 : abRankEma)
                  << ", final spacing " << featureTracker.settings.featureDist << " px."
                  << std::endl;
    }

    return 0;
}

VisionMeasurement convertGIFTFeatures(const std::vector<GIFT::Feature>& GIFTFeatures, const double& stamp) {
    VisionMeasurement measurement;
    measurement.stamp = stamp;
    std::transform(
        GIFTFeatures.begin(), GIFTFeatures.end(),
        std::inserter(measurement.camCoordinates, measurement.camCoordinates.begin()),
        [](const GIFT::Feature& f) { return std::make_pair(f.idNumber, f.camCoordinatesEigen()); });
    return measurement;
}
