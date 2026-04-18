# 29295C Push-Back

29295C Push-Back is a VEX motion-control project focused on making autonomous drive, turn, and point-to-point motion easier to tune and more repeatable. The codebase includes a reusable motion API, PID control, odometry tracking, and HTML-based visualizers for tuning and simulation.

## What It Does

- Provides short autonomous calls like drive, turn, and s_drive.
- Uses odometry, inertial heading, and optional GPS / distance fusion to improve motion accuracy.
- Centralizes drivetrain PID tuning in a single configuration header.
- Includes browser-based visualizers for motion behavior and PID response.

## Live Demo

Experienceable link: publish the repository to GitHub Pages and paste the live URL here.

Recommended deployment target:

- GitHub Pages from the repository root, using `index.html` as the entry point.

To publish with the included workflow:

1. Push the repo to GitHub.
2. Enable GitHub Pages using the `GitHub Actions` source.
3. The workflow in `.github/workflows/pages.yml` will deploy the static site.

## Screenshot or Video

Upload at least one project screenshot or a short demo video, then link it here. This still has to be hosted or attached in your submission flow.

Suggested format:

- Screenshot: add a hosted image URL or repository asset link.
- Video: add a hosted video URL if you have a walkthrough or demo recording.

## AI Use Declaration

AI assistance was used during development to help draft and refine project documentation and supporting text. All final project content should be reviewed before submission.

## Project Structure

- `index.html` and `pid_visualizer.html` provide the browser visualizers.
- `src/main.cpp` contains the robot entry point.
- `include/` contains the motion, odometry, and PID interfaces.
- `docs/` contains architecture and system notes.
- `template/` contains starter files for future projects.

## Submission Checklist

- Live URL added to the README.
- Screenshot or video uploaded and linked.
- Project description explains the project clearly.
- AI use declaration is present in the README.
