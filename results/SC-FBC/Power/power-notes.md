# jendela A — idle 60 detik (board nyala, tanpa beban): reset counter, tunggu, catat
# jendela B — run live utuh:
python3 scripts/run_k230_live_campaign.py --seq advio-14 --force
#    reset counter tepat sebelum menekan enter / saat proses mulai,
#    catat counter saat terminal mencetak "Finished with exit code 0"
#    (hentikan pencatatan SEBELUM adb pull dimulai)
# jendela C — idle 60 detik: catat lagi (validasi idle stabil)

# idle
5.194V
0.163A
0.846W
00:01:30
000004 mAh
000021 mWh

# run live
5.191V
00:02:10
000007 mAh
000036 mWh

# idle
5.194V
0.163A
0.846W
00:01:30
000004 mAh
000021 mWh