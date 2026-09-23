# Pipeline diagnostics

Small scripts used to find the real-time latency problems described in
`docs/superpowers/specs/2026-09-23-realtime-pipeline-findings.md`. They need a
running system (or a replay) and a roscore.

| Script | What it answers |
|---|---|
| `ratemon.py OUT.txt` | What rate do time surfaces and poses actually come out at? Logs each message's stamp and arrival time for `/image_representation_TS_l`, `..._TS_r` and `/esvo2_tracking/pose_pub`. Use this instead of `rostopic hz`, which reports nothing useful under `use_sim_time`. |
| `pcmon.py OUT.txt` | How big is the local map, and how often is it published? One line per `/esvo2_mapping/pointcloud_local2`: stamp, point count, and count of points marked visible. |
| `pcdepth.py OUT.txt` | How far away is the map? Per published map: stamp, size, and the median and 10th/90th percentile point distance. Used to show depth inflating under load. |
| `retype_vector.py LEFT.bag RIGHT.bag IMU.bag OUT.bag` | Makes VECtor's bags playable here. Its `prophesee_event_msgs/EventArray` has the same layout as `dvs_msgs/EventArray` (same md5), so messages are rewritten raw, without deserialising, under the topic names the launch files expect. |
| `rebag_events.py IN.bag OUT.bag [chunk_s]` | Splits `dvs_msgs/EventArray` messages into `chunk_s` slices (default 5 ms), keeping the IMU topics. Datasets that batch events at 30 Hz cap the time surface rate, because a surface is only rendered when an event message arrives. |

Typical use, alongside a replay:

```shell
python3 ratemon.py /tmp/rates.txt &
PLAYRATE=1 ../replay_eval.sh /tmp/run bag.bag "<topics>" <launch> ...
kill %1
```

Replay the same bag at 0.5× and at 1× and compare: a change that helps at 0.5×
but not at 1× is limited by compute or latency, not by the algorithm.
