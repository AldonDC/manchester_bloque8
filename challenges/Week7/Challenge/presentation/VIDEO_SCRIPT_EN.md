# Video Recording Script (English) — MCR2 Final Challenge

Target length: **90 seconds**. Casual narration over the timelapse footage.
Use the PDF presentation as the intro (slides 1–4) and outro (slides 13–14).

---

## 0:00 – 0:08 · Title card
> "Hi, I'm Alfonso. This is my Final Challenge for the Manchester Robotics
> course — autonomous Puzzlebot navigation, built from scratch with NumPy
> and ROS 2."

Show **slide 1** (title) for 3 seconds, then **slide 2** (mission) for 5 seconds.

---

## 0:08 – 0:20 · Part 1 in action (EKF)

Switch to RViz recording. Robot is moving slowly. Zoom on the **covariance ellipse**.

> "Part 1 is the Extended Kalman Filter. The robot fuses wheel encoders
> with ArUco marker observations. The pink ellipse around the robot is the
> 95% confidence region — when no marker is visible the ellipse grows;
> when an ArUco lands it shrinks immediately. RMSE is around 7 millimeters
> with 99% marker detection."

---

## 0:20 – 0:35 · A* planning & path smoothing

Show **slide 8** (occupancy grid) or screenshot of RViz with green path.

> "Part 2 plans a global path with A* on a 5-centimeter grid, inflated by
> the robot radius for safety. The raw path has sharp corners, so I smooth
> it with a Catmull-Rom spline — that gives a C1-continuous curve the
> robot can follow with Pure Pursuit."

---

## 0:35 – 1:15 · Navigation timelapse (×4 speed)

Real timelapse footage of the full loop: WP0 → WP1 → WP2 → WP3 → WP0.

> "Pure Pursuit picks a lookahead point 30 centimeters ahead on the
> smoothed path and steers toward it. Maximum linear velocity is 18
> centimeters per second. The trajectory closes the loop visiting all
> four waypoints, satisfying the PDF rules: no straight legs, no
> co-directional consecutive legs, every chamber is circumnavigated."

While the robot moves, point out:
- Green line = planned path
- Red dots = LiDAR scan filtered
- Pink ellipse = EKF uncertainty
- The robot stays in the corridor — no wall contact

---

## 1:15 – 1:25 · Closing

Show final frame: robot at WP0, "MISIÓN COMPLETADA · LAP 1" in terminal.

> "The robot completes one full closed lap, satisfies both Part 1 and
> Part 2 of the PDF, plus the optional integration. Everything implemented
> from scratch — NumPy and Python standard library only. Thanks for watching."

Show **slide 14** (Thank you) for 3 seconds.

---

# How to record the timelapse

## 1) Record the screen at normal speed

Open two terminals:

```bash
# Terminal 1: launch the simulation
./run.sh part2-astar

# Terminal 2 (after Gazebo and RViz are up): record desktop
# Pick ONE of these recorders — whatever you have installed:

# Option A: gnome screen recorder (Ctrl+Alt+Shift+R) — easiest, MP4 output
# Option B: OBS Studio — full-screen capture
# Option C: ffmpeg X11 grab (no install needed)
ffmpeg -y -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
       -c:v libx264 -preset ultrafast -pix_fmt yuv420p \
       challenges/Week7/Challenge/presentation/raw_run.mp4
```

Let the robot complete the full lap (about 75 seconds). Press `q` in
the ffmpeg terminal (or Ctrl+C) when "MISIÓN COMPLETADA" appears.

## 2) Speed up to ×4 timelapse

```bash
ffmpeg -i raw_run.mp4 -filter:v "setpts=0.25*PTS" \
       -an -c:v libx264 -preset slow -crf 22 \
       timelapse_4x.mp4
```

A 75-second run becomes 19 seconds. Adjust the multiplier:
- `setpts=0.5*PTS` → ×2 speed (faster but still readable)
- `setpts=0.25*PTS` → ×4 speed (recommended)
- `setpts=0.125*PTS` → ×8 speed (very fast, good for long runs)

## 3) Combine with the slides (optional)

If you want the final video to include the LaTeX slides:

```bash
# Export the PDF to images first
cd challenges/Week7/Challenge/presentation
pdftoppm -r 150 presentation.pdf slide -png   # creates slide-01.png ... slide-18.png

# Then in your video editor (Kdenlive, OpenShot, DaVinci Resolve, iMovie):
#   - Drop slide-01.png and hold for 3 s (title)
#   - Drop slide-02.png and hold for 5 s (mission)
#   - Drop timelapse_4x.mp4 with voiceover narration
#   - Drop slide-14.png and hold for 3 s (thanks)
```

## 4) Faster path with ffmpeg only (no editor)

If you want to skip the video editor entirely and just concatenate the
timelapse with a fixed-duration intro/outro slide:

```bash
# Make a 3-second video from one slide PNG:
ffmpeg -loop 1 -t 3 -i slide-01.png -vf "scale=1920:1080" \
       -c:v libx264 -pix_fmt yuv420p -tune stillimage intro.mp4

ffmpeg -loop 1 -t 3 -i slide-14.png -vf "scale=1920:1080" \
       -c:v libx264 -pix_fmt yuv420p -tune stillimage outro.mp4

# Concatenate intro + timelapse + outro
cat > concat.txt <<EOF
file 'intro.mp4'
file 'timelapse_4x.mp4'
file 'outro.mp4'
EOF
ffmpeg -f concat -safe 0 -i concat.txt -c copy final_video.mp4
```

---

# Quick tips for recording

- **Close the editor + browser** before launching: load average drops
  from ~12 to ~4, RViz becomes much smoother for the recording.
- **RViz top-down view** (already configured in `ekf.rviz`): the
  TopDownOrtho view is the cleanest angle for the timelapse.
- **Gazebo window** can stay open in a corner — viewers like seeing
  both the simulator and RViz at the same time.
- **Audio**: record narration separately with a clean mic and overlay
  in the editor. Trying to narrate live with Gazebo running is painful.
- **One-take vs edited**: a single un-cut take (~75 s real, sped to 19 s
  in post) is usually faster than trying to edit multiple takes.
