// Example motion calls

// Drive a straight distance (inches)
drive(24.0, 80, 2500);

// Turn to absolute heading (degrees)
turn(90.0, 2000);

// Simultaneous drive + heading target:
// Go to X=36 in, Y=24 in, finish at heading=45 deg.
s_drive(36.0, 24.0, 45.0, 85, 4000);

// With quick inline tuning override for one path:
s_drive(48.0, 48.0, 90.0, 90, 5000,
        2.5, 0.0, 0.03,
        1.4, 0.0, 0.08,
        true);
