#!/usr/bin/env python3
"""Serial, resumable paced replay evidence on the actual K230."""
import argparse
import bisect
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'paper-vio-eqvio/scripts'))
from run_k230_deployment_evidence import (AUTO, POLICY, adb, cmd, controller,
    distribution, evaluate, push, readcsv, save, sha, shell)
from run_sota_repro_matrix import EUROC_SHORT, resolve_dataset_path, write_groundtruth
import numpy as np

PILOT = [('euroc', 'mh02'), ('euroc', 'v203'), ('tum-vi', 'room3')]
FLAGS = {'eqvio': [], 'static15': ['--maxFeaturesOverride', '15'],
         'static5': ['--maxFeaturesOverride', '5'], 'static10': ['--maxFeaturesOverride', '10'],
         'ours': POLICY + AUTO + ['--adaptiveBudgetAutoFloor', '--adaptiveBudgetAutoHold'],
         'base-policy': POLICY + AUTO,
         'floor-only': POLICY + AUTO + ['--adaptiveBudgetAutoFloor'],
         'hold-only': POLICY + AUTO + ['--adaptiveBudgetAutoHold'],
         'ours-warm-g40': POLICY + AUTO + ['--adaptiveBudgetAutoFloor','--adaptiveBudgetAutoHold','--adaptiveBudgetAutoHoldWarmup']}


def matrix(profile='primary'):
    jobs = []
    def add(ds, seq, arm, seed, block, stress=False, full_start=False):
        job = dict(dataset=ds, sequence=seq, resolution='half', arm=arm, seed=seed,
                   logging=True, block=block, stress=stress, full_start=full_start)
        job['id'] = f'{block}/{ds}/{seq}/{arm}/s{seed}'
        jobs.append(job)
    if profile=='fixed-sweep':
        for ds,seq in PILOT:
            for arm in ['static5','static10']:
                add(ds,seq,arm,0,'fixed-sweep')
        return jobs
    if profile=='tumvi-five-arm':
        for seq in [f'room{i}' for i in range(1,7)]:
            for arm in ['eqvio','base-policy','floor-only','hold-only','ours']:
                add('tum-vi',seq,arm,0,'tumvi-five-arm')
        return jobs
    if profile=='euroc-five-arm':
        for seq in EUROC_SHORT.values():
            for arm in ['eqvio','base-policy','floor-only','hold-only','ours']:
                add('euroc',seq,arm,0,'euroc-five-arm')
        return jobs
    if profile in ['fixed-all17','floor-all17']:
        for ds, sequences in [('euroc', list(EUROC_SHORT.values())),
                              ('tum-vi', [f'room{i}' for i in range(1, 7)])]:
            for seq in sequences:
                add(ds,seq,'static10' if profile=='fixed-all17' else 'floor-only',0,profile)
        return jobs
    if profile=='hold-diagnosis':
        for ds,seq in PILOT:
            for seed in range(3):
                arms=['floor-only','ours','ours-warm-g40']
                arms=arms[seed:]+arms[:seed]
                for arm in arms:add(ds,seq,arm,seed,'hold-diagnosis')
        return jobs
    for stress in [False, True]:
        for ds, seq in PILOT:
            for seed in range(3):
                # Rotate order to avoid always measuring one arm on a warmer board.
                arms = ['eqvio', 'static15', 'ours']
                for arm in arms[seed:] + arms[:seed]:
                    add(ds, seq, arm, seed, 'stress' if stress else 'pilot', stress)
    for ds, seq in PILOT:
        for seed in range(3):
            for arm in ['base-policy', 'floor-only', 'hold-only']:
                add(ds, seq, arm, seed, 'ablation')
    for ds, sequences in [('euroc', list(EUROC_SHORT.values())),
                          ('tum-vi', [f'room{i}' for i in range(1, 7)])]:
        for seq in sequences:
            if (ds, seq) not in PILOT:
                for arm in ['eqvio', 'ours']:
                    add(ds, seq, arm, 0, 'main')
    for seed in range(3):
        for arm in ['eqvio', 'ours']:
            add('euroc', 'mh01', arm, seed, 'initialization', full_start=True)
    return jobs


def realtime_metrics(rows, deadline_ms=50):
    arrival = np.array([float(r['arrival_s']) for r in rows])
    start = np.array([float(r['start_s']) for r in rows])
    finish = np.array([float(r['finish_s']) for r in rows])
    assert len(rows) and np.all(np.isfinite([arrival, start, finish]))
    assert np.all(np.diff(arrival) > 0) and np.all(start >= arrival - 1e-6)
    assert np.all(finish >= start) and np.all(np.diff(finish) >= 0)
    latency = (finish-arrival)*1000
    # Implicit lossless FIFO: count timestamp-scheduled camera arrivals still pending.
    backlog = np.maximum(0, np.searchsorted(arrival, start, side='right') - np.arange(len(rows)) - 1)
    output_backlog = np.maximum(0, np.searchsorted(arrival, finish, side='right') - np.arange(len(rows)) - 1)
    return dict(frames=len(rows), arrival_to_output_ms=distribution(latency),
                processing_ms=distribution((finish-start)*1000),
                dispatch_lateness_ms=distribution((start-arrival)*1000),
                deadline_ms=deadline_ms, deadline_miss_fraction=float(np.mean(latency > deadline_ms)),
                pending_camera_at_dispatch=distribution(backlog),
                pending_camera_at_output=distribution(output_backlog),
                camera_median_dt_s=float(np.median(np.diff(arrival))) if len(rows)>1 else None,
                final_output_lag_ms=float(latency[-1]),
                backlog_slope_frames_per_s=float(np.polyfit(arrival, backlog, 1)[0]) if len(rows)>1 else None)


def resilient_shell(command, timeout=60, shell_fn=shell):
    try:
        return shell_fn(command,timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f'WARNING: adb shell timed out after {timeout}s; retrying once: {command}',
              file=sys.stderr,flush=True)
        return shell_fn(command,timeout=timeout)


def remote_checksums(directory, shell_fn=shell):
    command='cd '+shlex.quote(str(directory))+' && sha256sum *'
    return resilient_shell(command,timeout=120,shell_fn=shell_fn)


def archive(out):
    summaries = []
    for seq in EUROC_SHORT:
        for arm, stem in [('eqvio', 'current_eqvio_baseline'), ('ours', 'current_eqvio_ours_sc_afb')]:
            folder = ROOT/'result/k230/euroc-mav'/seq
            timing, logs = folder/(stem+'_timing.csv'), folder/(stem+'_eqvio_filterlog.csv')
            if not timing.exists():
                continue
            rows = readcsv(timing)
            item = dict(dataset='euroc', sequence=seq, arm=arm, source=str(timing),
                        timing_sha256=sha(timing), scope='offline pipeline, not arrival-to-output',
                        latency_ms=distribution([float(r['pipeline_latency_us'])/1000 for r in rows]))
            if logs.exists():
                records=readcsv(logs)
                item['applied_cap']=distribution([int(r['max_features_applied']) for r in records])
                item['accepted']=distribution([int(r['n_meas_used']) for r in records])
                item['update_applied_fraction']=float(np.mean([int(r['update_applied']) for r in records]))
                if arm=='ours':
                    trace=controller(records, arm, 'euroc', 'half')
                    path=out/'archive'/f'{seq}_controller.csv'
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with path.open('w') as f:
                        w=csv.DictWriter(f, fieldnames=list(trace[0]));w.writeheader();w.writerows(trace)
                    item['controller_switches']=sum(r['switched'] for r in trace)
            summaries.append(item)
    for arm in ['eqvio', 'eqvio_ours']:
        for room in range(1,7):
            folder=ROOT.parent/f'results/k230-tumvi-pgm-half/20261006_082312/{arm}/room{room}'
            path=folder/'raw/timing.csv'
            if path.exists():
                summaries.append(dict(dataset='tum-vi',sequence=f'room{room}',arm=arm,
                    source=str(path),timing_sha256=sha(path),scope='offline pipeline; legacy TUM controller differs',
                    latency_ms=distribution([float(r['total'])*1000 for r in readcsv(path)])))
    save(out/'archive_summary.json', summaries)


def refresh(out, jobs):
    runs=[json.loads(p.read_text()) for p in (out/'runs').glob('**/run.json')]
    save(out/'summary.json', dict(planned=len(jobs), completed=len(runs), runs=runs))
    header=['block','dataset','sequence','arm','seed','verdict','ate_m','rpe_1s_m',
            'latency_p95_ms','latency_p99_ms','deadline_miss_fraction','backlog_max','cpu_mean_pct']
    with (out/'summary.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=header);w.writeheader()
        for r in runs:
            rt=r['realtime']
            w.writerow(dict(block=r['block'],dataset=r['dataset'],sequence=r['sequence'],arm=r['arm'],
                seed=r['seed'],verdict=r['verdict'],ate_m=r['accuracy']['ate_rmse_m'],
                rpe_1s_m=r.get('rpe_1s_individual',{}).get('rpe_rmse_m'),
                latency_p95_ms=rt['arrival_to_output_ms']['p95'],latency_p99_ms=rt['arrival_to_output_ms']['p99'],
                deadline_miss_fraction=rt['deadline_miss_fraction'],
                backlog_max=rt['pending_camera_at_dispatch']['max'],cpu_mean_pct=r['cpu_pct']['mean']))
    return runs


def stage(out, remote, ds, seq):
    host, calib, longname=resolve_dataset_path(ds, seq, 'half')
    assert host.is_dir() and calib.is_file(), (host, calib)
    name=('euroc_'+longname) if ds=='euroc' else 'tumvi_'+seq
    camera='/root/dataset/PGM-half/'+name+'/mav0/cam0'
    imu='/root/dataset/PGM-quarter/'+name+'/mav0/imu0'
    case=out/'cases'/ds/seq;case.mkdir(parents=True,exist_ok=True)
    target=remote+'/input'
    cmd('mkdir','-p',target+'/mav0/cam0',target+'/mav0/imu0')
    cmd('cp','-r',camera+'/data',target+'/mav0/cam0/',timeout=240)
    cmd('cp',camera+'/data.csv',target+'/mav0/cam0/data.csv')
    cmd('cp',imu+'/data.csv',target+'/mav0/imu0/data.csv')
    push(calib,target+'/intrinsics.yaml')
    shutil.copy2(calib,case/'intrinsics.yaml')
    assert write_groundtruth(host,case/'groundtruth.tum')
    resilient_shell('cd '+shlex.quote(camera)+' && sha256sum data.csv data/* > '+shlex.quote(target+'/camera.sha256'),timeout=240)
    resilient_shell('cd '+shlex.quote(target+'/mav0/cam0')+' && sha256sum -c '+shlex.quote(target+'/camera.sha256')+' > '+shlex.quote(target+'/verified.log'),timeout=240)
    adb('pull',target+'/camera.sha256',case/'camera.sha256',timeout=120)
    camera_csv=cmd('cat',target+'/mav0/cam0/data.csv')
    (case/'camera.csv').write_text(camera_csv+'\n')
    timestamps=[int(r[0])*1e-9 for r in csv.reader(camera_csv.splitlines()) if r and not r[0].startswith('#')]
    imu_csv=cmd('head','-n','3',target+'/mav0/imu0/data.csv')
    imu_first=next(int(r[0])*1e-9 for r in csv.reader(imu_csv.splitlines()) if r and not r[0].startswith('#'))
    assert np.all(np.diff(timestamps)>0)
    meta=dict(camera_source=camera,imu_source=imu,camera_manifest_sha256=sha(case/'camera.sha256'),
              imu_sha256=cmd('sha256sum',target+'/mav0/imu0/data.csv').split()[0],
              calibration_sha256=sha(calib),frames=len(timestamps),first_stamp=min(timestamps[0],imu_first),
              camera_median_dt_s=float(np.median(np.diff(timestamps))))
    save(case/'dataset.json',meta)
    return target,case,timestamps,meta


def run(out, jobs):
    lock=(out/'campaign.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Also serialize with other instances of this campaign on the host.
    boardlock=Path('/tmp/k230-realtime-board.lock').open('w');fcntl.flock(boardlock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    remote='/tmp/k230-realtime-'+out.name
    store='/root/k230-realtime-'+out.name
    cmd('mkdir','-p',remote,store)
    ps=cmd('ps')
    assert not any('eqvio_opt' in line for line in ps.splitlines()), 'Another estimator is active'
    for name in ['eqvio_opt','board_run.sh','stress.sh','euroc.yaml','tum-vi.yaml']:
        push(out/'frozen'/name,store+'/'+name)
    cmd('chmod','+x',store+'/eqvio_opt')
    if not (out/'hardware.json').exists():
        save(out/'hardware.json',dict(description=resilient_shell('uname -a; cat /proc/cpuinfo; cat /proc/meminfo; cat /sys/devices/system/cpu/online'),
             binary_sha256=cmd('sha256sum',store+'/eqvio_opt').split()[0]))
    active=None
    for job in jobs:
        folder=out/'runs'/job['id']
        if (folder/'run.json').exists():
            continue
        save(out/'state.json',dict(status='preparing',current=job['id'],planned=len(jobs),host_pid=os.getpid()))
        key=(job['dataset'],job['sequence'])
        if key!=active:
            cmd('rm','-rf',remote+'/input')
            input_path,case,timestamps,data=stage(out,remote,*key)
            active=key
        folder.mkdir(parents=True,exist_ok=True)
        shutil.copy2(case/'groundtruth.tum',folder/'groundtruth.tum')
        offset=20.55 if job['sequence']=='mh01' and not job['full_start'] else 0
        expected=sum(t>=data['first_stamp']+offset for t in timestamps)
        output=remote+'/runs/'+job['id']
        binary=store+'/eqvio_opt'
        config=out/'frozen'/f"{job['dataset']}.yaml"
        remote_config=store+'/'+job['dataset']+'.yaml'
        if (case/'config.yaml').exists():
            config=case/'config.yaml';remote_config=store+'/case-config.yaml'
            push(config,remote_config)
        args=[input_path+'/',remote_config,'--mode','asl','--output',output,
              '--camera',input_path+'/intrinsics.yaml','--timing','--limitRate','0','--pacedReplay',
              '--seed',str(job['seed']),'--featureDistOverride','80','--filterLog',output+'/filterlog.csv']+FLAGS[job['arm']]
        if offset:args+=['--start',str(offset)]
        meta={**job,'expected_frames':expected,'start_offset_s':offset,'input':data,
              'campaign_binary_sha256':sha(out/'frozen/eqvio_opt'),
              'config_sha256':sha(config),
              'command':[binary]+args,'board_before':resilient_shell('cat /proc/uptime; cat /sys/class/thermal/thermal_zone0/temp')}
        launch=folder/'launch.json'
        started=bool(resilient_shell('test -f '+shlex.quote(output+'/run.log')+' && echo yes || true'))
        if started:
            if not launch.exists():raise RuntimeError('Remote output without local provenance: '+output)
            meta=json.loads(launch.read_text())
        else:
            save(launch,meta)
            cmd('mkdir','-p',output)
            cmd('start-stop-daemon','-S','-b','-m','-p',output+'/runner.pid','-x','/bin/sh','--',store+'/board_run.sh',output,binary,*args)
            if job['stress']:
                cmd('start-stop-daemon','-S','-b','-m','-p',output+'/stress_runner.pid','-x','/bin/sh','--',store+'/stress.sh',output)
        print('RUN '+job['id'],flush=True)
        save(out/'state.json',dict(status='running',current=job['id'],planned=len(jobs),
             remote_output=output,host_pid=os.getpid()))
        deadline=time.monotonic()+3600
        while True:
            time.sleep(3)
            try:
                marker=shell('test -s '+shlex.quote(output+'/EXITCODE')+' && cat '+shlex.quote(output+'/EXITCODE')+' || true')
                if marker:
                    if not job['stress'] or shell('test -s '+shlex.quote(output+'/stress_done')+' && echo yes || true'):break
            except (RuntimeError,subprocess.TimeoutExpired):
                if time.monotonic()>deadline:raise
                continue
            if time.monotonic()>deadline:raise TimeoutError('Run did not finish; preserved '+output)
        raw=folder/'raw';raw.mkdir(exist_ok=True)
        adb('pull',output+'/.',str(raw)+'/',timeout=300)
        hashes=remote_checksums(output)
        for line in hashes.splitlines():
            h,name=line.split(None,1)
            assert sha(raw/name.strip())==h, name
        meta['board_after']=resilient_shell('cat /proc/uptime; cat /sys/class/thermal/thermal_zone0/temp')
        result=evaluate(folder,meta)
        if result['verdict']=='evaluation_error':raise RuntimeError(result['evaluation_error'])
        result['paced_estimator_fps']=result.pop('offline_estimator_fps',None)
        rows=readcsv(raw/'realtime.csv')
        result['realtime']=realtime_metrics(rows,data.get('deadline_ms',50))
        assert len(rows)==result['processed_frames']
        result['realtime']['expected_input_frames']=expected
        result['realtime']['unprocessed_input_frames']=expected-len(rows)
        result['realtime']['policy']='lossless timestamp-scheduled replay; pending cameras inferred from schedule; no live capture'
        if job['stress']:
            events=readcsv(raw/'stress.csv')
            epoch=float((raw/'replay_epoch.txt').read_text())
            on=float(events[0]['monotonic_s'])-epoch;off=float(events[1]['monotonic_s'])-epoch
            result['stress_events_relative_s']={'on':on,'off':off}
            phases={'before':lambda t:t<on,'loaded':lambda t:on<=t<off,'after':lambda t:t>=off}
            result['stress_phase_latency_ms']={name:distribution([(float(r['finish_s'])-float(r['arrival_s']))*1000 for r in rows if include(float(r['arrival_s']))]) for name,include in phases.items()}
        if key[0]=='euroc' and job['block']=='pilot' and job['seed']==0 and job['arm'] in ['eqvio','ours']:
            longname=next(k for k,v in EUROC_SHORT.items() if v==key[1])
            stem='current_eqvio_'+('baseline' if job['arm']=='eqvio' else 'ours_sc_afb')
            prior=ROOT/'result/k230/euroc-mav'/longname/(stem+'_trajectory.tum')
            old=np.loadtxt(prior);new=np.loadtxt(folder/'trajectory.tum')
            result['offline_archive_pose_parity']=bool(old.shape==new.shape and np.allclose(old,new,rtol=0,atol=1e-5))
        save(folder/'run.json',result)
        refresh(out,jobs)
        print(f"DONE {job['id']} {result['verdict']} ATE={result['accuracy']['ate_rmse_m']:.4f} p99={result['realtime']['arrival_to_output_ms']['p99']:.2f}ms misses={result['realtime']['deadline_miss_fraction']:.3%}",flush=True)
        if result['exit_code']!=0 or not result['complete_input']:raise RuntimeError('Input or process failure; raw artifacts preserved')
        # Local raw outputs and checksums are durable before reclaiming board tmpfs.
        cmd('rm','-rf',output)
    # Release the staged sequence before a dependent campaign can start.
    cmd('rm','-rf',remote+'/input')
    save(out/'state.json',dict(status='complete',planned=len(jobs),completed=len(refresh(out,jobs))))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--detach',action='store_true');ap.add_argument('--self-check',action='store_true')
    ap.add_argument('--profile',choices=['primary','fixed-sweep','fixed-all17','floor-all17','hold-diagnosis','tumvi-five-arm','euroc-five-arm'],default='primary')
    ap.add_argument('--after',type=Path)
    args=ap.parse_args();jobs=matrix(args.profile)
    if args.self_check:
        assert len(matrix())==115 and len({r['id'] for r in matrix()})==115
        assert len(matrix('fixed-sweep'))==6
        assert len(matrix('fixed-all17'))==17
        assert len(matrix('floor-all17'))==17
        assert len(matrix('hold-diagnosis'))==27
        tumvi=matrix('tumvi-five-arm')
        assert len(tumvi)==30 and len({r['id'] for r in tumvi})==30
        assert {r['sequence'] for r in tumvi}=={f'room{i}' for i in range(1,7)}
        assert {r['arm'] for r in tumvi}=={'eqvio','base-policy','floor-only','hold-only','ours'}
        assert all(r['dataset']=='tum-vi' and r['resolution']=='half' and r['seed']==0 for r in tumvi)
        euroc=matrix('euroc-five-arm')
        assert len(euroc)==55 and len({r['id'] for r in euroc})==55
        assert {r['sequence'] for r in euroc}==set(EUROC_SHORT.values())
        assert {r['arm'] for r in euroc}=={'eqvio','base-policy','floor-only','hold-only','ours'}
        assert all(r['dataset']=='euroc' and r['resolution']=='half' and r['seed']==0 for r in euroc)
        rows=[dict(arrival_s=str(t),start_s=str(t+d),finish_s=str(t+d+.02)) for t,d in [(0,0),(.05,.06),(.10,.04),(.15,0)]]
        m=realtime_metrics(rows)
        assert m['deadline_miss_fraction']==.5 and m['pending_camera_at_dispatch']['max']==1
        assert '--adaptiveCovariance' not in FLAGS['ours']
        assert FLAGS['base-policy']==POLICY+AUTO
        assert FLAGS['floor-only']==POLICY+AUTO+['--adaptiveBudgetAutoFloor']
        assert FLAGS['hold-only']==POLICY+AUTO+['--adaptiveBudgetAutoHold']
        assert FLAGS['ours']==POLICY+AUTO+['--adaptiveBudgetAutoFloor','--adaptiveBudgetAutoHold']
        print('115 primary runs; 30 TUM-VI and 55 EuRoC five-arm runs; controller flags and paced timing checked');return
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if not (out/'manifest.json').exists():
        frozen=out/'frozen';frozen.mkdir()
        shutil.copy2(ROOT/'build-k230-paced/eqvio_opt',frozen/'eqvio_opt')
        shutil.copy2(ROOT/'scripts/board_run.sh',frozen/'board_run.sh')
        shutil.copy2(ROOT/'scripts/board_stress_realtime.sh',frozen/'stress.sh')
        shutil.copy2(ROOT/'src/main_opt.cpp',frozen/'main_opt.cpp')
        shutil.copy2(Path(__file__),frozen/'runner.py')
        for ds,name in [('euroc','EQVIO_config_EuRoC_stationary.yaml'),('tum-vi','EQVIO_config_TUM_VI.yaml')]:
            shutil.copy2(ROOT/'configs'/name,frozen/(ds+'.yaml'))
        shutil.copy2(ROOT.parent/'notes/k230-realtime-protocol.md',out/'protocol.md')
        if args.profile=='fixed-sweep':
            with (out/'protocol.md').open('a') as f:
                f.write('\n## Fixed-cap supplement\n\nSix seed0 normal paced controls, caps5/10 on MH02/V203/room3, after the primary campaign. Added because pilot MH02 reaches only 13 accepted observations and static15 does not constrain it. Selection uses observation counts, not accuracy outcomes. Compare with primary seed0 baseline/SC-AFB/static15; this supplement is diagnostic, not three-seed repeatability.\n')
        if args.profile=='fixed-all17':
            with (out/'protocol.md').open('a') as f:
                f.write('\n## Fixed10 full benchmark supplement\n\nSeventeen NEW seed0 normal 20 Hz paced runs: all eleven EuRoC sequences and six TUM-VI rooms, mono cam0 half PGM, fixed feature cap10. Same binary, dataset, feature distance80, config, logger and MH01 start20.55s as the primary baseline/SC-AFB arms. No observation adaptation or covariance override. Compare every sequence, including divergence and deadline tails, against primary seed0 baseline and SC-AFB; reuse those completed reference runs. Timing was collected in separate campaigns, not randomized contemporaneous pairs. The three earlier pilot Fixed10 runs are retained as prior results, not added to the17-sequence aggregate.\n')
        save(out/'matrix.json',jobs)
        save(out/'manifest.json',dict(files={p.name:sha(p) for p in frozen.iterdir()},
             profile=args.profile,after=str(args.after.resolve()) if args.after else None,
             deadline_ms=50,scope='timestamp-paced camera/IMU replay, estimator output-ready; no sensor-driver or downstream transport latency'))
        archive(out)
    if args.detach:
        with (out/'campaign.log').open('a') as log:
            p=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--output',str(out),
                '--profile',args.profile]+(['--after',str(args.after.resolve())] if args.after else []),
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        save(out/'host_process.json',dict(pid=p.pid,log=str(out/'campaign.log')))
        print('Detached campaign pid='+str(p.pid));return
    try:
        if args.after:
            save(out/'state.json',dict(status='waiting',dependency=str(args.after.resolve()),planned=len(jobs),host_pid=os.getpid()))
            while json.loads((args.after/'state.json').read_text()).get('status')!='complete':
                predecessor=json.loads((args.after/'host_process.json').read_text())
                if not Path(f"/proc/{predecessor['pid']}").exists():raise RuntimeError('Preceding campaign stopped; do not take the board')
                time.sleep(10)
        run(out,jobs)
    except Exception as error:
        save(out/'error.json',dict(error=repr(error),time=time.time()))
        raise


if __name__=='__main__':main()
