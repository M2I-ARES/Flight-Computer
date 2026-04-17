//We're (almost) ready?
/*
MO-NAV-EH
Moats Navigation-Computer Eh?
This is a square root-spherical simplex unscented kalmann filter with augmented process noise
this was designed around an MPU 9150, BMP 280, and GPS with altitude,and precision data.  
Data is output to TX via Serial1 pins (POS,VEL,ACCELEROMETER BIASES,INERTIAL->BODY QUATERNION,GYRO SENSOR BIASES,MAG SENSOR PITCH BIAS,MAG SENSOR YAW BIAS,PRESSURE SENSOR BIAS)

TO-DO
Ellipsoidal Magnetometer Calibration sequence (MODERATE)
Altimeter Updating (HIGH) (in progress...)
EGM-2008 Onboard modeling (in progress...)
WMM Onboard modeling (LOW)
^XYZ geoMag?
fix barometer function (HIGH) (in progress)

magnetometer unexpected lockout? (MODERATE)
Mach number compensation/lockout (HIGH) (in progress...)

GPS data discard (LOW) Done!? (maybe additional overrides but I trust the isValid() statement)
Manual Mode (MODERATE)
Write/test transmission functions (HIGH) (in progress)

Testing! (HIGH) In Progress...

COMPLETE

Double check/set correct rates for IMU (HIGH)
magnetometer bias modeling(HIGH)
re-verify magnetometer function (HIGH) 
GPS data discard (LOW)
SD Logging (HIGH) 

TESTS TO-DO
GPS verification (live) Done! (Needs large scale testing)
Altimeter verification (spoofed and live)

TESTS COMPLETED

Orientation initialization/Mag Field rotation (spoofed and live) Done!
Orientation updating (live) Done!
INS verification (live) Done! (within bounds)

FUTURE WORKS

Weather effect modeling/estimation
payload integration (bluetooth?)
ground tracking station support (additional observer)
tachometer observer

*/
/*
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢻⡙⢮⡜⡹⣛⠿⢿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣓⢮⠧⡝⢦⡙⢦⡑⣎⡑⢢⠍⢞⡹⣿⣿⣿⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⢳⡝⣎⣳⡙⢦⡙⢦⡱⢌⡜⣡⢊⠴⣀⢃⠻⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡝⣧⡛⣜⢦⡙⢦⡙⢦⡑⠎⡔⢢⢁⠒⢄⠊⡔⠀⢏⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡻⣵⠹⡜⢦⡙⢦⡙⠦⡘⠬⡐⠡⠌⠌⡂⠰⢈⠐⡈⢢⠹⢿⣿⣿⣿⣿⣿⡛⢾⡱⢯⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣞⡵⣋⠷⣙⠦⡙⢦⠱⣁⠣⡑⡌⡑⣈⠰⢀⠅⣂⠐⠰⡀⢌⠢⠹⢧⠉⢆⡡⢎⠥⣛⠬⣇⢻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣫⡕⣫⢼⡙⢮⡑⣎⠱⣊⠱⢄⠣⡑⠤⢁⠄⠡⢈⡐⠠⠉⠆⡑⢂⠡⠒⠤⢋⠆⡱⢊⠼⣈⠷⣌⣳⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣱⢇⢯⡵⢫⣜⢣⡕⣎⡱⢌⠚⡄⢣⠘⡄⠣⠌⢆⡡⠤⣁⠎⡐⡐⢂⠌⡱⢈⠣⡌⡱⣉⠖⣩⢖⡱⣎⢿⣞⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢷⣙⢮⣚⡜⡳⣬⢗⡼⣠⢃⢎⠱⡈⢆⠡⢄⠡⡈⠄⠠⠡⠄⠢⠑⡈⠆⡘⠤⣉⠒⣡⠒⣡⠚⡔⢮⡱⢎⡷⣫⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢧⡏⢶⡱⣺⢵⣧⣿⣿⣿⣿⣾⣷⣷⣾⣶⣬⠱⡐⡌⠲⣀⢂⠡⠐⡠⢐⡐⠒⡄⡓⠤⡉⢆⡹⡘⢦⣙⢾⡸⣝⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢇⡻⢴⡩⢷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣵⢣⢆⠣⢎⡱⣑⠦⣜⣣⣜⣥⣓⣜⡰⣂⡙⢦⣛⠶⣱⡹⣞⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢎⡳⣃⠞⡭⢛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡷⢪⠱⣎⣵⣿⣿⣾⣿⣿⣿⣿⣿⣷⣿⣿⣦⣹⢏⢶⣹⢯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⡟⣿⣿⣟⣮⠳⣥⢚⡐⢃⠲⡘⠿⢿⣿⣿⣿⣿⡯⢹⠿⡟⢻⢻⡇⡁⠎⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣮⢳⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣹⢚⣿⣿⣿⣎⡗⣆⠣⠜⢢⡑⡘⠔⡢⢌⡽⣿⣿⠷⠟⡰⢈⡅⢮⣷⠐⢌⣿⡟⡻⢿⡏⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⢬⢳⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⠼⣽⣻⣿⣿⡜⢦⡙⢎⠥⢂⠱⣈⠐⠤⠐⢠⠀⠌⡐⢡⠒⡎⣵⡆⢈⠴⣿⣯⠔⢢⠻⣮⣹⠿⢡⣻⣿⡛⠦⢩⠹⡘⣎⢷⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢎⡹⢽⣿⣿⣟⢦⡹⢌⠎⡥⢃⠔⡈⢆⠉⠤⢈⠂⣅⣣⡙⠴⣸⢁⠊⢾⣋⢿⣏⠆⡐⠩⢙⠛⡛⢛⡉⢇⡘⢄⢣⠱⣺⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣎⠱⡌⣿⣿⣿⣦⠹⣌⠚⡤⢉⡒⠄⠂⠌⡐⣠⣶⡿⣉⢎⠱⣚⠄⣉⢾⡜⡺⣿⡜⡀⢃⠆⢂⡁⠆⡌⢰⢈⠎⡔⣫⢷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⢿⣿⢿⣿⣿⣻⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣽⣿⣿⣿⣯⢳⣬⢓⡌⡡⠜⡠⢉⠴⠞⢣⢿⣗⡰⢌⡵⡌⡐⢠⢋⡳⢱⠘⣷⡌⠠⠘⠠⡐⢌⠰⢃⠎⡜⡸⣵⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿
  ⣿⣻⣿⣯⣿⢿⣾⣿⢿⣾⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡗⣮⢣⡜⡰⢱⠘⡌⢢⠉⢆⠹⣿⣿⣿⣷⣼⣴⣣⣾⣵⣧⣮⣿⡇⠱⣈⠱⡐⢌⠲⣉⠦⣍⣷⣿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿
  ⣿⣿⣷⣿⣻⣿⣿⣾⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡷⣿⣿⡜⣧⢚⡵⢣⢃⣼⡶⣛⢢⡡⣍⣏⠻⢿⣿⣿⣿⣿⣿⣿⣿⠟⣌⠱⢄⡓⡌⢆⡳⣌⠾⣼⣿⣿⣯⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣾⣿⣿⣿⣾⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣷⡹⣎⠳⣌⠧⣿⣯⣶⡿⢯⡱⣄⣦⣉⣦⣙⡛⡛⣍⠛⢻⢮⣽⣢⢏⡲⡘⣬⢣⡗⣞⡿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿
  ⣿⣿⣿⣽⣾⣿⣿⣿⣻⣿⣿⣿⣿⣿⣯⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣞⣷⡻⣭⢳⠎⡜⣻⡿⠿⣿⣿⣿⣿⣿⡟⠛⠿⠿⣿⣿⣿⣷⣮⣽⣿⣶⣽⡷⣊⢷⡹⢾⣽⣿⣿⣿⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿⡿⣿⣿⣽⣿⣿⣾⣿⡵⣫⣝⡳⣌⡱⢏⡒⠥⡛⣿⣿⣽⣻⠶⣶⠶⡦⣴⢿⣿⣿⣿⣛⠻⣍⢲⣙⡮⣽⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣻⣿⣿
  ⣿⣿⡟⠿⠻⣿⠟⡛⣿⠛⣙⢻⣿⡉⢍⡹⣿⣿⢿⣿⡿⣿⣯⣿⣿⣿⣿⣷⢳⣎⠷⡸⣼⢣⠜⡰⠱⣌⠛⡿⣥⣙⡬⣑⣫⣽⣯⣿⣿⡟⢤⢓⡰⢪⣼⡿⣭⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿
   ⠿⢀⠣⣼⣿⠰⡁⢮⣿⡀⢎⣳⣿⡤⢡⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣳⢎⣻⡕⣫⠞⣴⢡⠓⡌⠳⠞⣭⢛⣿⢿⡻⣽⠾⢟⣡⢚⡴⢣⢞⣽⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿
  ⡉⠔⣈⢳⣿⣿⢠⢑⢺⣿⡅⢣⢾⣿⣇⢣⣹⣿⣿⣿⣿⣽⣿⣿⡟⣾⣿⣿⣿⣯⢿⣞⡥⣛⡝⣆⠣⢌⠣⢍⡰⢋⡜⢣⠱⣈⠳⢌⠲⣯⣾⢏⡾⣿⡽⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
  ⠰⣉⠤⡉⢿⣿⡆⢎⢺⣿⡇⢣⢾⣿⡇⢶⣸⣿⣿⣿⣿⡿⣟⠯⣞⣽⣿⣿⣿⣿⣿⣯⣷⣹⡜⢦⠣⡌⠓⡌⠱⣌⠌⡥⢃⡔⢣⠎⣵⡿⣿⣾⣿⣿⣽⣿⣿⣿⣿⣟⢿⢻⠻⣝⠻⢏⡟⣻⢟⡿⢿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿
  ⠡⣾⣧⢱⣈⣻⣧⣊⣼⣿⣧⣷⣾⣿⣿⣷⠿⡟⢯⡝⣲⢹⣏⠷⣚⡼⢿⣿⣿⣿⣿⣿⣿⣿⣿⣭⢷⣌⣣⠌⢣⠜⣸⠰⢩⡘⣤⣻⣿⣻⣿⣿⣿⣿⣿⣿⣟⣿⣿⡟⣎⠥⣋⠤⢋⠔⡨⠐⣉⠐⠂⢤⣈⠛⡟⠿⣿⣿⣿⣿⣿
  ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡟⡽⢮⠻⡙⢇⠛⡅⢺⣏⡞⣱⢚⣬⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣽⣣⣾⣵⡿⣶⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⣜⣯⢿⣿⡜⠲⣉⠦⢡⠘⠠⡁⢄⡉⢌⠲⡌⢢⢉⠖⡰⢂⠭⡙⡟
  ⣿⣿⣿⣿⣿⣿⣿⣿⡿⡟⢯⢣⠙⡡⢃⢂⠱⣈⠆⡓⢈⠜⣿⣌⢣⢎⠴⣋⠽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣝⡻⣜⢮⣻⡿⣼⢃⠤⡉⢂⠍⠒⠌⡄⡒⣬⠓⡈⢂⠡⠊⠥⡉⠦⡑⡘
  ⣿⣿⣿⣿⠟⡛⢩⠉⢆⠉⠄⢂⠡⡐⠢⢌⠂⡁⢆⠐⠠⠈⣿⣆⢣⠚⣆⢣⡙⡼⣛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣯⡝⣎⢳⡹⢮⣿⣱⡿⢈⢂⠱⠈⢄⠣⢰⣬⠿⠁⠌⠠⢀⢂⠑⣂⠑⢢⠁⠆
  ⣿⠟⡫⢐⠢⠘⠤⠉⡄⢊⠌⠄⠣⠌⡁⢆⠢⢑⡈⢂⠁⠢⢹⣧⢊⠵⣈⠦⣑⠦⡙⢮⢽⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢯⡗⡧⢽⡌⢧⣙⣿⣳⡿⢁⠊⠄⢂⣡⣮⡾⠟⠉⡄⠣⢈⠐⡌⢢⠑⠢⢉⢂⠩⠐
  ⢌⠒⡡⠌⢂⠩⡐⢡⠈⠆⡘⡈⠔⠨⢐⠢⡁⢆⠐⡠⠘⡀⠌⢿⡌⢖⡡⠞⡤⢃⡹⢌⢲⢣⠻⣽⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣏⡟⠧⡝⡼⣡⢚⣵⠿⣷⡟⡀⠂⠌⡰⡱⠋⠅⡐⢠⠃⠔⡀⢆⡱⠌⣁⠊⡐⢠⠂⠂⠅
  ⠢⡑⢌⡘⠄⣃⠰⠁⡌⢂⠡⠐⣈⠰⢁⠢⢁⠌⡂⠄⠡⡐⠠⠘⢿⡢⡙⢷⡸⢥⡒⣌⠊⡕⢫⢜⡹⣞⠿⣿⣿⣿⣿⣿⢿⢯⡳⡝⢮⠜⣣⠝⣢⣵⡿⢋⣾⠟⠠⢀⠃⠜⠠⠁⠌⡰⢈⠁⢌⠢⡁⠆⡐⠰⢀⠂⡁⢂⠌⡈⠐
  ⡔⡌⢆⡘⠰⣀⠣⠘⠠⣁⠊⠔⡀⢂⠂⡡⠌⢢⠐⡈⠐⡀⠂⠄⠘⣿⡄⢃⠍⡳⢿⣦⡑⢌⡀⠎⡔⡈⣳⣿⣿⡿⣿⣿⣿⣬⠱⣙⢪⢱⢣⣿⡾⢋⠔⣱⡟⡌⢁⠢⠘⠈⠄⣁⠊⠄⠡⢈⠆⠃⠄⡡⠌⡁⠆⢄⠡⢂⠐⠠⡁
  ⠘⡈⠆⡑⠂⠄⢂⠡⠡⡀⠎⡐⠠⠂⠄⢡⠘⡀⠆⡁⠂⠄⠡⢈⠀⠸⣷⠈⠄⡑⢊⠹⢿⣶⣈⠒⣠⣿⣿⣛⠟⡿⣿⣿⣿⣿⣷⣌⣲⣿⡿⠋⡔⡁⢊⣿⠍⡀⠄⢂⠡⢈⡐⢠⠈⠄⢡⢂⠌⡐⢂⠅⡒⢈⠐⡠⠂⠤⢉⡒⠄
  ⠐⡀⠒⠠⠁⡌⠀⠆⠡⠐⡐⠠⢁⠂⠌⡀⢆⠡⠂⠄⡁⢂⠐⠠⢀⠀⠻⣧⠀⡑⢈⠡⢊⠹⢿⣶⣿⢿⣿⣿⣯⣴⣹⣿⣿⣿⣿⣿⠟⡡⢂⠱⢠⠁⣺⡏⠄⡐⢈⠂⠔⠡⠐⡀⠂⠌⠰⢈⠠⡐⠌⢒⠈⠄⠂⠄⢡⠘⢦⠉⡜
  ⢂⠡⢊⢁⠒⠠⠉⢄⠡⠒⠠⢁⠂⠌⡀⢐⡈⠤⢁⠂⡐⢀⠂⡐⢀⠂⠀⢻⡄⢀⠃⡐⡀⠃⢌⣹⣿⠠⢻⣿⣿⣿⣿⣿⣿⡿⠛⣷⡌⠐⠠⠃⢄⠢⣿⠁⠂⠔⠠⠈⠄⡁⠂⠄⠡⢌⠡⠀⢂⠵⢈⠂⠌⠠⢁⠊⠤⣙⠦⡑⠤
  ⠀⠂⠄⡂⠌⠠⢁⠂⠤⢁⠂⠄⡈⠐⡀⢂⠰⠐⡀⢂⠐⡀⢂⠀⢂⠠⠁⡀⢒⠠⠈⢀⠠⢑⣴⠟⣿⠄⣹⣿⣿⣿⣿⣿⠟⠡⢁⠌⠛⣮⡔⠠⣀⢼⡏⠀⠌⡐⠀⡁⠂⠄⠁⡈⢐⠂⠠⡈⡜⢂⠡⠈⠄⡁⠂⡌⠲⣍⠒⡌⡐
  ⠀⡁⢂⠐⡈⠐⠠⢈⠐⠠⢈⠐⡀⠡⠐⡀⢂⠡⠐⢂⠐⡀⢂⠈⠄⡐⠠⠀⠌⣂⠉⠡⣰⡿⠃⠄⠸⣇⣿⣿⣷⣿⣿⣿⠈⡀⠌⠘⡐⠨⠙⢄⠀⡸⠀⠌⠠⠐⡀⠐⠈⠠⠐⠠⢁⠂⠡⢰⡉⠆⡈⠐⠠⢀⠡⡘⡵⣊⠱⢠⠑
  ⠀⡐⠀⠂⠄⡁⢂⠂⡁⢂⡐⠠⠐⢀⠡⠐⡀⢂⠁⢂⠐⡀⢂⠈⠄⠠⠁⢂⠐⠠⣌⡿⢋⠠⠐⣈⢱⣿⢋⣿⣿⣿⣿⣿⡒⡐⡈⢄⠠⠁⠌⢀⠣⠄⡁⠂⠁⢂⠐⠈⢀⠁⠌⡐⠠⠀⢥⡓⡌⠐⠀⡁⠂⢄⠢⣙⠶⣁⠃⢆⡉
  ⠀⡐⠠⢁⠂⡐⠠⢈⠐⠠⢀⠁⠂⠄⢂⠡⠐⠠⠈⠄⠠⠀⠄⢂⠈⠄⠂⠄⠂⠱⢋⢁⠂⡄⢣⢀⣿⣿⣾⣿⣿⣿⣿⣿⣷⠱⣍⠞⣦⢁⡂⠄⠀⡐⢀⠂⠌⠀⡀⠡⠀⡈⠀⠄⠁⣈⠶⡑⡈⢀⠡⠐⡈⢄⠣⣝⠎⡤⢉⠆⡐
  ⠀⠄⡁⢂⠐⠠⢁⠠⢈⠐⠠⠈⡐⠈⠄⠂⡁⢂⠡⠈⠄⡁⠂⠄⢂⠐⠠⠈⠄⠡⢀⠂⡘⢀⠂⣼⣿⣿⣿⡽⣿⣿⢯⣿⣿⠡⢌⠚⡠⢣⠘⣀⠂⠐⠀⠄⠂⠐⠀⡐⠠⠐⠈⢀⠢⣌⡗⠡⠐⠀⡀⠂⠔⣨⢓⣎⠳⣀⠣⡘⠠
  ⢀⠂⠀⠄⡈⠐⡀⠄⠂⡀⠂⢁⠠⠈⠄⠡⠐⠠⢀⠁⢂⠀⠌⠀⢂⠈⠄⠡⠐⡀⠂⠄⡐⢄⣚⣿⣿⣿⡿⢿⡷⣻⣿⣿⣿⡇⢠⠘⡐⠄⢣⠐⡌⠰⢀⠂⡈⢀⠁⠠⠀⠂⠁⢄⡺⡴⢉⠠⠀⠡⢀⠁⠎⣔⣫⢌⡓⢄⠣⡐⠡
  ⡀⠌⢀⠂⠄⠁⠀⠄⡁⢀⠂⠠⠐⢈⠠⠁⠌⠐⠠⠈⠄⡈⠄⢁⠂⢈⠠⠁⠄⡐⢈⠔⣘⢮⣿⣿⢿⣿⠿⣱⣾⣿⣿⣿⣿⣏⠦⣱⠸⣌⠧⡱⣈⠱⡈⡔⢠⠂⡈⠄⠡⢈⡐⢮⣱⢋⠄⠠⠁⠂⠄⡘⡘⢦⢏⡲⢌⠢⢑⠠⠁
  ⡐⢀⠂⠀⢂⠈⢀⠂⠄⠠⠐⠀⠡⠀⢂⠈⠄⠈⠄⢁⠂⠄⡈⠀⠄⢂⠠⢈⠔⡠⢍⢮⡵⢫⣾⣿⡾⣯⣿⣿⣿⣿⣿⠳⣿⣧⠊⠵⢫⡜⣣⢓⠬⣡⠓⡬⡑⣎⠰⡈⠔⡡⣜⢣⡳⠌⠀⠄⡁⠌⢠⢡⡙⢮⡚⠴⡈⢆⠡⢂⠁
  ⢈⠄⡐⠈⢀⠠⠀⡀⠂⢁⠠⠈⠄⠡⠀⢂⠈⠄⡈⠠⠐⠠⢀⠡⠐⡀⠆⣌⢢⠵⣎⠷⣚⣽⣿⣫⣾⣿⣿⣿⣿⡟⢧⣿⣿⣷⢈⠰⢁⠎⡵⣊⠦⡑⢎⡔⠳⣌⠳⡈⢆⡱⢬⢳⠡⠈⢀⠂⠄⡈⢄⠲⣍⢧⡙⠦⡑⡈⠰⢀⠂
  ⠊⠰⢀⠁⠂⠄⠡⠐⠁⠂⢀⠐⠀⠂⡁⢀⠂⠠⠀⡁⠠⠁⠄⢂⠡⡘⢆⢧⣋⠾⣜⣭⠳⣼⣿⣿⣿⣿⣿⡿⢳⣸⣿⣿⣿⣿⢀⠂⢌⡘⠴⣉⠖⣩⠒⡬⢓⡌⡳⡁⢆⡹⢎⠧⢁⠂⠄⠈⠠⠐⡌⡳⢎⠶⣉⠖⠡⡐⢁⠂⠄
*/
//Math stuff
#include <ArduinoEigen.h>
#include <cmath>
#define StateNum 15
#define ProcNum 6
#define L StateNum+ProcNum
#define N L+2
const double W0=0.2;//Tune this, weight of mean (noiseless) sigma point
const double W1=(1-W0)/(L+1);//weights of all the other ones
//Sensor stuff
const double gyroScale=((M_PI/180)/65.5);//conversion from Raw Data to Rad/s
const double accelScale=9.81/2048.0f;//Conversion from Raw Data to m/s/s
#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C
#include <Wire.h>
#include <I2Cdev.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
const int chipSelect = BUILTIN_SDCARD;//For Teensy 4.1 onboard SD
Eigen::Vector3d mag;//magnetometer reading
Eigen::Matrix<double,6,1> rates;//raw body rates from IMU
Eigen::Quaterniond inerToBody;//Mean/Predicted Orientation Quaternion
Eigen::Quaterniond nextInerToBody;//Predicted k+1 Quaternion using mean gyr bias
TinyGPSPlus gps;//take a guess
//Time stuff
unsigned long lastMillis;//time in ms at the end of the last timestep
double Time=0;//timer variable for how long the system has been on
double timer;//timer varible for measurement update frequency
double logTimer;//timer variable for datalogging
double calibtimer;//CHANGE temporary timer for calibration
//Debugging Stuff
#define DEBUG true
#define printPropogations false //prints every sigma point propogation
#define printPropogationProb true //prints covariance update math
#define printMeas true//prints the sigma points and matricies used in the measure function
//weather stuff
#define lapseRate 1
#define Rdry 287.05287//J/kg/K
#define Rvap 461.51
#define LaunchHeight 50
#define SemiMajor 6378137
#define SemiMajorSq 6378137.0*6378137.0
#define SemiMinorSq 6356752.314245*6356752.314245
#define ecc 1/298.257223563
double Undulation;
double groundTemp;//Ground temperature in deg K
double groundPres;//Ground Pressure in Pa
double groundAlt;//Launch altitude  in m
double QNH;
//UKF stuff
Eigen::Matrix<double,StateNum,StateNum> Sx;
Eigen::Matrix<double, StateNum,1> y_mean;
Eigen::MatrixXd Y;
Eigen::Matrix<double,StateNum,1> State;//xPos,yPos,zPos, xVel,yVel,zVel, xAccelBias,yAccelBias,zAccelBias, xAngErr,yAngErr,zAngErr, xGyroBias,yGyroBias,zGyroBias, Pitch Mag Bias(Rad),Yaw Mag Bias(Rad),Pressure Sensor Bias
//Proccess variables accelerometer Noise, gyro Noise (NEXT TO BE ADDED) Accel Bias Noise, Gyro Bias Noise, Noise in Mag Bias Pitch, Noise in Mag Bias Yaw
Eigen::Matrix<double,L,N> SigPoints;
Eigen::Matrix<double,N,1> w;
Eigen::Matrix<double,3,1> magDir;//true magnetic vector in inertial frame
Eigen::Matrix<double,N,1> W;
Eigen::Matrix<double,ProcNum,ProcNum> Sw;
Eigen::Matrix<double,3,3> RGps;
Eigen::Matrix<double,3,3> RMag;
Eigen::Matrix<double,6,6> RStill;
Eigen::Matrix<double,StateNum,1> x_mean;
double Grav=9.80665;//Standard gravitational accecleration m/s/s
Eigen::Vector3d inertialDown;//down (gasp)
Eigen::Vector3d inertialNorth;
Eigen::Vector3d inertialEast;
double dt;//previous loop's time step in s
Eigen::Vector3d initialCoords;//what it says it is (XYZ ofc)                                                                                                                                                                                                                                                                                                                       Easter Egg
//I2C stuff 
void writeByte(uint8_t addr, uint8_t reg, uint8_t val) {//I2C stuff
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
uint8_t readByte(uint8_t addr, uint8_t reg) {//More I2C stuff
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0XdF;
}
bool readBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {//Even More I2C stuff
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (uint8_t i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}
bool getMag(Eigen::Ref<Eigen::Matrix<double,3,1>> Mag){

  if(DEBUG){
    Serial.println("Getting Magnetometer...");
  }

  //Ask for measurement
  writeByte(MAG_ADDR, 0x0A, 0x01);

  //Wait for data to be ready
  unsigned long t0 = millis();
  uint8_t st1 = 0;

  while (millis() - t0 < 100) {
    st1 = readByte(MAG_ADDR, 0x02);
    if (st1 & 0x01) break;
    delay(1);
  }

  if (!(st1 & 0x01)) return false;

  //Read data + ST2 together
  uint8_t buf[7];
  if (!readBytes(MAG_ADDR, 0x03, 7, buf)){
    Serial.println("Magnetometer Read Failed");
    return false;
  }

  // Check overflow FIRST
  if (buf[6] & 0x08){
    Serial.println("Magnetometer Overflow");
    return false;
  }
  int16_t mx = (int16_t)((uint16_t(buf[1]) << 8) | buf[0]);
  int16_t my = (int16_t)((uint16_t(buf[3]) << 8) | buf[2]);
  int16_t mz = (int16_t)((uint16_t(buf[5]) << 8) | buf[4]);

  //Mag axis remapping to accel axis
  Mag(1) = mx;
  Mag(0) = my;
  Mag(2) = -mz;

  if(DEBUG){
    Serial.println("Magnetometer Got!");
    printMatrix(Mag.transpose());
  }

  return true;
}
void init9150() {//resets and turns on MPU, sets rates, and enables magnetometer bypass
  Wire.setClock(100000);//Setting I2C clock to 100kHz
  //Reset MPU
  writeByte(0x68, 0x6B, 0x00);
  delay(500);//gotta have these here or it gets finnicky
  //Wake IMU
  writeByte(0x68, 0x6B, 0x01);
  delay(500);
  //Disable I2C Master mode
  writeByte(0x68, 0x6A, 0x00);
  delay(500);
  //Enable bypass so we can talk to magnetometer
  writeByte(0x68, 0x37, 0x02);
  delay(500);
  //Accelerometer configuration PG 6 of https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9150-Register-Map.pdf for register map
  writeByte(0x68, 0x1C, 0x18);
  delay(500);
  //gyro configuration
  writeByte(0x68, 0x1B, 0x8);
  delay(500);
  /*
  
  useful Hex codes for accel/gyro configs
  FS_SEL  HEX   Gyro Scale(deg/s)  Gyro Res   Accel Scale(g) Accel Scale
  0       0x0   131.0              +-250      16384          +-2
  1       0x8   65.5               +-500      8192           +-4
  2       0x10  32.8               +-1000     4096           +-8
  3       0x18  16.4               +-2000     2048           +-16

  ConvertedData = RawData/scale
  */   
  writeByte(MAG_ADDR,0x0B,0x01);
  //reset Magnetometer
  delay(500);
  //WHO_AM_I request for MPU
  if(DEBUG){
    Serial.println("MPU WHO_AM_I");
    Serial.println(readByte(0x68,0x75),HEX);
    //WHO_AM_I request for AK8975
    Serial.println("AK8975 WHO_AM_I");
    Serial.println(readByte(MAG_ADDR, 0x00), HEX);
    Serial.println("BYPASS MODE CHECK");
    Serial.println(readByte(0x68,0x37), HEX);
    Serial.println("USER CONTROL CHECK");
    Serial.println(readByte(0x68,0x6A),HEX);
  }

}
template<typename DerivedS, typename DerivedV>//No idea but it wsa crashing without this
void cholUpdate(Eigen::MatrixBase<DerivedS>& S,  const Eigen::MatrixBase<DerivedV>& x_in, double w){//Chat Wrote this ngl
  if(DEBUG){
    Serial.println("Doing Cholesky....");
    Serial.println("X");
    printMatrix(x_in.transpose());
    Serial.println("X norm");
    Serial.println(x_in.norm());
    Serial.println("weight");
    Serial.println(w);
  }
    auto& Sm=S.derived();
    Eigen::VectorXd x=x_in;
    const int n = Sm.rows();
    double sign = (w >= 0.0) ? 1.0 : -1.0;
    double Alpha = std::sqrt(std::abs(w));

    x *= Alpha;

    for (int k = 0; k < n; ++k)
    {
        double r = std::sqrt(Sm(k,k)*Sm(k,k) + sign * x(k)*x(k));
        if(isnan(r)){
          r=0.02;//prevents super small variances from blowing up filter
        }
        double c = r / Sm(k,k);
        double s = x(k) / Sm(k,k);
        printMatrix(x.transpose());
        Sm(k,k) = r;

        for (int j = k + 1; j < n; ++j)
        {
            Sm(k,j) = (Sm(k,j) + sign * s * x(j)) / c;
            x(j) = c * x(j) - s * Sm(k,j);
        }
    }
    if(DEBUG){
      Serial.println("Cholseky Done!");
      Serial.println("Sm");
      printMatrix(Sm);
      Serial.println("x");
      printMatrix(x.transpose());
    }
}
Eigen::Vector3d avgRot(Eigen::Matrix<double,3,Eigen::Dynamic> params,Eigen::Matrix<double,Eigen::Dynamic,1>weights){
    //finds average rotation from a set of modified rodriguez parameters (MRP's)
    //params are 3xn MRP corresponding to nx1 weights vector
    //finds average rotation using Eigendecomposition sum of outer products method
    Eigen::Matrix<double,4,4> accum=Eigen::Matrix<double,4,4>::Zero();
    for(int i=0; i<params.cols(); i++){
      double sqNorm=params.col(i).squaredNorm();
      if(sqNorm>1){
        params.col(i)-=params.col(i)/sqNorm;
      }
      Eigen::Matrix<double,4,1> Quat;
      //turning MRP into Quaternion
      Quat(0)=(1-sqNorm)/(1+sqNorm);
      Quat.segment(1,3)=2*params.col(i)/(1+sqNorm);
      if(Quat(0)<0){
        Quat=-Quat;
      }
      accum+=Quat*Quat.transpose()*weights(i);
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(accum);
    //find largest eigenValue and extract the eigenvector as average quaternion
    int maxIndex;
    eig.eigenvalues().maxCoeff(&maxIndex);
    Eigen::Matrix<double,4,1> qavg=eig.eigenvectors().col(maxIndex);
    //converting back to MRP
    Eigen::Vector3d avg=qavg.segment(1,3)/(1+qavg(0));
    if(avg.norm()>1){
      avg=-avg/avg.squaredNorm();
    }
    return avg;
  }
Eigen::Matrix<double,StateNum,1> Propogate(Eigen::Ref<Eigen::VectorXd> state,Eigen::Ref<Eigen::VectorXd> proc){
  Eigen::Vector3d modRodParam=(state.segment(9,3));//inertial frame orientation offset (from expected orientation)
  if(modRodParam.norm()>1){
    modRodParam=-modRodParam/modRodParam.squaredNorm();
  }
  Eigen::Matrix<double,3,1> accel=rates.segment(0,3)-state.segment(6,3)+proc.segment(0,3);//accel rates - accel biases + accel noise
  Eigen::Vector3d bodyRate=rates.segment(3,3)+proc.segment(3,3)-state.segment(12,3);//rates that differ from the mean point's rate
  Eigen::Quaterniond thisInerToBody;//Quaternion Correction from IF->Body small angle rotation error
  Eigen::Quaterniond IFRotError;
  IFRotError.w()=(1-modRodParam.squaredNorm())/(1+modRodParam.squaredNorm());
  IFRotError.vec()=2*modRodParam/(1+modRodParam.squaredNorm());
  Eigen::Quaterniond update;//Quaternion Correction from deviation from mean rates
  update.w()=1;
  update.vec()=0.5*bodyRate*dt;
  update.normalize();
  //update angular error with QuatCorrect*inerToBody.conj()=dQ
  //update orientation with inerToBody*angErr=dQ

  thisInerToBody=(IFRotError*inerToBody)*update;//body frame angular error correction
  IFRotError=thisInerToBody*nextInerToBody.conjugate();//rotate back with the nominal quaternion to find the quaternion error
  Eigen::Matrix<double,3,3> Rmat=((thisInerToBody).toRotationMatrix().transpose());
  //Eigen::Matrix<double,3,3> Rmat=((inerToBody.toRotationMatrix().transpose()));
  Eigen::Matrix<double,3,1> IFAccel=Rmat*accel+inertialDown*Grav;
  Eigen::Matrix<double,3,1> Vel=state.segment(3,3);
  Eigen::Matrix<double,3,1> Pos=state.segment(0,3)+(Vel*dt)+(IFAccel*dt*dt/2);
  Vel=Vel+(IFAccel*dt);
  Eigen::Matrix<double,StateNum,1> PropState=state;//CHANGE Double check when this is done
  if(printPropogations){
    Serial.println("-------------Propogation Model-------------");
    Serial.println("Initial Quaternion");
    Serial.print(inerToBody.w());
    Serial.print(", ");
    printMatrix(inerToBody.vec().transpose());
    Serial.println("Process Noise");
    printMatrix(proc);
    Serial.println("Body Rates");
    printMatrix(rates.segment(3,3).transpose()*180/M_PI);
    Serial.println("Accelerations");
    printMatrix(accel.transpose());
    Serial.println("Rmat");
    printMatrix(Rmat);
    Serial.println("IF Accelerations");
    printMatrix(IFAccel.transpose());
    Serial.println("IF Velocity");
    printMatrix(Vel.transpose());
    Serial.println("IF Position");
    printMatrix(Pos.transpose());
    Serial.println("Error Quaternion");
    Serial.print(IFRotError.w());
    Serial.print(", ");
    printMatrix(IFRotError.vec().transpose());
    Serial.println("-------------------End--------------------");
  }
  PropState.segment(0,3)=Pos;
  PropState.segment(3,3)=Vel;
  PropState.segment(9,3)=IFRotError.vec()/(1+IFRotError.w());
  if(PropState.segment(9,3).norm()>1){
    PropState.segment(9,3)=-(PropState.segment(9,3))/PropState.segment(9,3).squaredNorm();
  }
  if(printPropogations){
    Serial.println("Propogated State");
    printMatrix(PropState.transpose());
  }
  return PropState;
}
void printMatrix(const Eigen::MatrixXd& m) {//does what it says, uses Eigen Matricies/vectors
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      Serial.print(m(i, j),3);
      Serial.print("\t");
    }
    Serial.println();
  }
}
void GenSigPoints(){//Generates Spherical Simplex Sigma Points
    // Scaled weights
    w=Eigen::Matrix<double,N,1>::Constant(W1);
    w(0) = W0;
    // building spherical simplex sigma point matrix Z
    Eigen::Matrix<double,L,N> Z = Eigen::Matrix<double,L,L+2>::Zero(L, L+2);
    // j = 1 initialization
    Z(0, 1) = -1.0/std::sqrt(2.0 * w(1));
    Z(0, 2) = 1.0/std::sqrt(2.0 * w(1));
    // recursive construction for j = 2 ... L
    for (int j = 2; j <= L; ++j){
        double denom = std::sqrt(j*(j+1)*w(1));
        // columns 1 ... j
        for (int i=1; i<=j; i++)
            Z(j-1,i)=-1.0/denom;
        // column j+1
        Z(j-1,j+1)=j/denom;
    }
    //CHANGE Clean this up later seperate simplex construction from sigpoint generation
    Eigen::Matrix<double,StateNum+ProcNum,1> RefState;
    RefState<<x_mean,Eigen::Matrix<double,ProcNum,1>::Zero(ProcNum);
    Eigen::Matrix<double,L,L> Sa;
    Sa.topLeftCorner(StateNum,StateNum)=Sx;
    Sa.topRightCorner(StateNum,ProcNum).setZero();
    Sa.bottomLeftCorner(ProcNum,StateNum).setZero();
    Sa.bottomRightCorner(ProcNum,ProcNum)=Sw;
    Z=Sa*Z;
    for (int i=0; i<N; i++){
    SigPoints.col(i)=RefState+Z.col(i);
    }
}
void sigPointsProp(){//Propogates model
  Eigen::Matrix<double,StateNum,1> x_i;
  Eigen::Matrix<double,StateNum,N> X_pred;
  Eigen::Matrix<double,ProcNum,1> w_i;
  Eigen::Quaterniond dQ;
  dQ.w()=1;
  dQ.vec()=(rates.segment(3,3)-x_mean.segment(12,3))*dt*0.5;
  nextInerToBody=inerToBody*dQ;
  for (int i = 0; i < N; ++i)
    {
        x_i = SigPoints.col(i).segment(0, StateNum);
        w_i = SigPoints.col(i).segment(StateNum, ProcNum);
        X_pred.col(i) = Propogate(x_i, w_i);
    }
  inerToBody=nextInerToBody;
    Eigen::Matrix<double,StateNum,1> x_mean = (X_pred*w.asDiagonal()).rowwise().sum();
    x_mean.segment(9,3)=avgRot(X_pred.block(9,0,3,N),w);//uses special averaging for weighted quaternions to find angular error
    State=x_mean.segment(0,StateNum);
    // QR on columns 1...L+1
    Eigen::MatrixXd A(StateNum, L+1);
    for (int i = 1; i < L+2; ++i){
        A.col(i-1) = sqrt(abs(w(i))) * (X_pred.col(i) - x_mean);
    }
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A.transpose());
    Sx = qr.matrixQR().topLeftCorner(StateNum, StateNum).triangularView<Eigen::Upper>();
    for (int i=0; i<StateNum; i++){
      Sx(i,i)=abs(Sx(i,i));
    }
    if(printPropogationProb){
      Serial.println("Propogated Points");
      printMatrix(X_pred);
      Serial.println("A matrix");
      printMatrix(A);
      Serial.println("Scaled Weights");
      printMatrix(w.transpose());
      printMatrix(Sx);
      Serial.println("x_mean");
      printMatrix(x_mean.transpose());
    }
    // Rank-1 update with 0th sigma point
    Eigen::VectorXd dx0 = (X_pred.col(0) - x_mean);
    cholUpdate(Sx, dx0, w(0));//Needs Scaling?
    if(printPropogationProb){
      Serial.println("dx0");
      printMatrix(dx0.transpose());
      Serial.println("Sx Post Cholesky");
      printMatrix(Sx);
      Serial.println("Sig Points Propogated");
    }

}
template<int Nm, typename Func>
void Meas(const Eigen::Matrix<double,Nm,1>& z,const Eigen::Matrix<double,Nm,Nm>& R, Func h,bool isVector){
  //SS-SR-UKF measurement function. if measurement is a Normalized vector, isVector should be true
  Eigen::Matrix<double,Nm,L+2> Y;
  for (int i = 0; i < L+2; ++i){
    
    Y.col(i) = h(SigPoints.col(i).segment(0,StateNum));
  }
  //something is failing in the magnetometer section. vector measurements are way higher than expected
  Eigen::Matrix<double,Nm,1>  y_mean = Y * w;
  if(isVector){
    y_mean.normalize();//makes y_mean into an averaged vector instead of direct average
  }
  Eigen::Matrix<double,Nm,Nm> Sy;
  Eigen::Matrix<double,Nm,N-1> A;
  for (int i=1; i<N; i++){
    A.col(i-1)=sqrt(abs(w(i)))*(Y.col(i)-y_mean);
  }
  Eigen::Matrix<double, Nm, (Nm+N-1)> B;
  B.leftCols(N-1)=A;
  B.rightCols(Nm)=R;
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(B.transpose());
  Sy=qr.matrixQR().topLeftCorner(Nm,Nm).template triangularView<Eigen::Upper>();
  Eigen::Vector<double,Nm>dy0=Y.col(0)-y_mean;
  if(printMeas){
    Serial.println("A matrix");
    printMatrix(A);
    Serial.println("R matrix");
    printMatrix(R);
    Serial.println("B matrix:");
    printMatrix(B);
    Serial.println("Sy matrix");
    printMatrix(Sy);
    Serial.println("Measurements (column vectors)");
    printMatrix(Y);
  }

  cholUpdate(Sy,dy0,w(0));
  Eigen::Matrix<double,StateNum,Nm> Pxy=Eigen::Matrix<double,StateNum,Nm>::Zero();
  for(int i=0; i<N; i++){
    Pxy+=w(i)*(SigPoints.col(i).segment(0,StateNum)-x_mean)*(Y.col(i)-y_mean).transpose();
  }//CHANGE wgat?
  Eigen::Matrix<double,Nm,Nm> SyT=Sy.transpose();
  Eigen::Matrix<double,Nm,StateNum> PxyT=Pxy.transpose().eval();//Pxy Transpose matrix
  Eigen::Matrix<double,Nm,StateNum> tmp=Sy.template triangularView<Eigen::Upper>().solve(PxyT);
  Eigen::Matrix<double,Nm,StateNum> Kt=Sy.transpose().template triangularView<Eigen::Lower>().solve(tmp);
  Eigen::Matrix<double,StateNum,Nm> K=Kt.transpose();
  x_mean+=K*(z-y_mean);
  if(printMeas){
    Serial.println("Pxy");
    printMatrix(Pxy);
    Serial.println("Z");
    printMatrix(z);
    Serial.println("Y_mean");
    printMatrix(y_mean);
    Serial.println("z-y");
    printMatrix(z-y_mean);
    Serial.println("SyT");
    printMatrix(SyT);
    Serial.println("PxyT");
    printMatrix(PxyT);
    Serial.println("tmp");
    printMatrix(tmp);
    Serial.println("Kt");
    printMatrix(Kt);
    Serial.println("K matrix");
    printMatrix(K);
    Serial.println("Post Measurement State");
    printMatrix(x_mean.transpose());
    Serial.println("Pre-Downdate Sx");
    printMatrix(Sx);
  }
  Eigen::Matrix<double,StateNum,Nm> U=K*Sy;
  for (int i=0; i<Nm; i++){
    cholUpdate(Sx,U.col(i),-1.0f);
  }
  if(printMeas){
    Serial.println("Post Cholesky Sy:");
    printMatrix(Sy);
    Serial.println("Pxy");
    printMatrix(Pxy);
    Serial.println("Post-Measurement State");
    printMatrix(x_mean.transpose());
    Serial.println("State Covariance");
    printMatrix(Sx);
  }
}
Eigen::Matrix<double,6,1> getINS(){//Returns 6 element Eigen vector [x,y,z]m/s/s,[x,y,z]rad/s //CHANGE TBJS
  if(DEBUG){
    Serial.println("Getting INS...");
  }
  uint8_t buffer[14];
  I2Cdev::readBytes(MPU_ADDR,0x3B,14,buffer);
  Eigen::Matrix<double,6,1> IMU;
  //CHANGE write this better
  IMU<<  (int16_t)((uint16_t(buffer[0])<<8)|buffer[1]),(int16_t)((uint16_t(buffer[2])<<8)|buffer[3]),(int16_t)((uint16_t(buffer[4])<<8)|buffer[5]),0.0f,0.0f,0.0f;
  IMU.segment(0,3)=IMU.segment(0,3)*accelScale;
  IMU(3)=(int16_t)((uint16_t(buffer[8])<<8)|buffer[9]);
  IMU(4)=(int16_t)((uint16_t(buffer[10])<<8)|buffer[11]);
  IMU(5)=(int16_t)((uint16_t(buffer[12])<<8)|buffer[13]);
  IMU.segment(3,3)=IMU.segment(3,3)*gyroScale;
  if(DEBUG){
  Serial.println("INS Got!");
  Serial.println("Data:");
  printMatrix(IMU.segment(0,3).transpose());
  printMatrix(IMU.segment(3,3).transpose()*180/M_PI);
  }
  return IMU;
}
double ISA(double alt){//ISA Standard temperature for indicated pressure altitude
  double temp;
  temp=288.15-2*alt/1000;
  return temp;
}
Eigen::Matrix<double,1,1> GetBaro(){
  Eigen::Matrix<double,1,1> geoAlt;
  return geoAlt;
}
Eigen::Matrix<double,3,1> PredGps(const Eigen::Ref<const Eigen::VectorXd>& state){
  Eigen::Matrix<double,3,1> PredMeas=state.segment(0,3);
  return PredMeas;
}
Eigen::Matrix<double,3,1> PredMag(const Eigen::Ref<const Eigen::VectorXd>& state){
  //Applying angle deviation from expected heading
  Eigen::Vector3d modRodParam=state.segment(9,3);
  if(modRodParam.norm()>1){
    modRodParam=-modRodParam/(modRodParam.squaredNorm());
  }
  Eigen::Quaterniond quatCorrect;
  quatCorrect.w()=(1-modRodParam.squaredNorm())/(1+modRodParam.squaredNorm());
  quatCorrect.vec()=2*modRodParam/(1+modRodParam.squaredNorm());
  Eigen::Matrix<double,3,1> Meas=(quatCorrect*inerToBody*magDir).normalized();
  //Applying Magnetometer Bias (15) Yaw (16) Pitch
  //CHANGE Include axis scaling factors
  /*
  double cY=cos(state(15));
  double sY=sin(state(15));
  double cP=cos(state(16));
  double sP=sin(state(16));
  //Matrix for 3-1 rotation
  Eigen::Matrix<double,3,3> BiasRmat;
  BiasRmat<<cY,-sY,0,cP*sY,cP*cY,-sP,sP*sY,sP*cY,cP;
  Meas=BiasRmat*Meas;
  */
  return Meas;
}
double PredBarometer(const Eigen::Ref<const Eigen::VectorXd>& state){
  double PredMeas=0;
  return PredMeas;
}
Eigen::Matrix<double,1,1> PredTachometer(const Eigen::Ref<const Eigen::VectorXd>& state){
  Eigen::Matrix<double,1,1> PredMeas;
  return PredMeas;
}
Eigen::Matrix<double,3,1> PredDown(const Eigen::Ref<const Eigen::VectorXd>& state){
  //CHANGE FUNCTION IS OUT OF DATE
  Eigen::Matrix<double,3,1> PredMeas;
  PredMeas=-state.segment(0,3);
  Eigen::Quaterniond angErr;
  angErr.w()=1;
  angErr.vec()=state.segment(9,3);
  PredMeas=(inerToBody*angErr).normalized()*PredMeas.normalized();
  return PredMeas;
}
Eigen::Matrix<double,6,1> PredStill(const Eigen::Ref<const Eigen::VectorXd>& state){
  Eigen::Matrix<double,6,1> PredMeas=state.segment(0,6);
  //PredMeas<<initialPos,Eigen::Vector3d::Constant(0);
  return PredMeas;
}
Eigen::Matrix<double,6,1> GetGps(){
  Eigen::Matrix<double,6,1> GPSDat;//[pos[x,y,z],covariance[x,y,z]]
  /*
  GPSDat<<6371000,0,0,10,5,5;//CHANGE SPOOFED DATA (At the equator at 0 lng)
  */
  while(Serial1.available()>0){
    gps.encode(Serial1.read());
  }
    if(gps.location.isUpdated()&&gps.location.isValid()&&gps.altitude.isValid()&&gps.hdop.isValid()){//move this to the update function
      Eigen::Vector3d GPSPos(gps.location.lat()*M_PI/180,gps.location.lng()*M_PI/180,gps.altitude.meters());//GPS coordinate data in radians
      Eigen::Vector3d GPSerr(gps.hdop.hdop(),gps.hdop.hdop(),2*gps.hdop.hdop());
      GPSerr=GPSerr*8;//using 8m as 95% confidence UERE
      double Ne=SemiMajor/(sqrt(1-ecc*ecc*sin(GPSPos(0))));//effective radius of WGS-84 ellipsoid
      GPSDat(0)=(Ne+GPSPos(2))*cos(GPSPos(0))*cos(GPSPos(1));
      GPSDat(1)=(Ne+GPSPos(2))*cos(GPSPos(0))*sin(GPSPos(1));
      GPSDat(2)=((1-ecc*ecc)*Ne+GPSPos(2))*sin(GPSPos(0));
      //CHANGE Check HDOP and VDOP here and see if reading needs to get discarded and update denied. do not go any further if so
      //^don't do this, lemme ponder
      Eigen::Matrix3d inertialRmat;//Matrix for projecting HDOP and VDOP into IF frame
      //Bring in down vector from get orientation and make it global here
      Eigen::Vector3d temp (-2*GPSDat(0)/SemiMajorSq,-2*GPSDat(1)/SemiMajorSq,-2*GPSDat(2)/SemiMinorSq);
      inertialDown=temp.normalized().cast<double>();
      Eigen::Vector3d Zax(0,0,1);//Z axis for geodectic North
      inertialNorth=(Zax-inertialDown*(Zax.dot(inertialDown))).normalized();//True North Projected onto Earth Tangent plane
      inertialEast=inertialDown.cross(inertialNorth).normalized();
      inertialRmat.row(0)=inertialNorth;
      inertialRmat.row(1)=inertialEast;
      inertialRmat.row(2)=inertialDown;
      GPSerr=(inertialRmat.cast<double>()*(GPSerr)).cwiseAbs();
      GPSDat.segment(3,3)=GPSerr;
      //Project HDOP and VDOP into IF and store it as the last 3 variables+put it into R_GPS
      File dataFile=SD.open("GPSData.txt",FILE_WRITE);
      for(int i=0; i<6; i++){
      dataFile.print(GPSDat(i));
      dataFile.print(",");
      }
      dataFile.println(Time);
      dataFile.close();
      RGps=GPSDat.cast<double>().segment(3,3).asDiagonal();
  }else{
    Serial.println("No GPS Update");
    GPSDat=Eigen::Matrix<double,6,1>::Constant(NAN);//this will trip a GPS update flag
  }
  return GPSDat;
}
void GPSUpdate(){
  if(DEBUG){
    Serial.println("Trying GPS update...");
    Serial.println("Getting GPS...");
  }
  Eigen::Matrix<double,6,1> Measurement;
  Measurement=GetGps();
  if(Measurement.hasNaN()){
    Serial.println("GPS Update Denied");
    return;
  }
  Measurement.segment(0,3)=Measurement.segment(0,3)-initialCoords;
  if(DEBUG){
    Serial.println("GPS got!");
    printMatrix(Measurement.transpose().cast<double>());
  }
  //Measurement Function
  Meas<3>(Measurement.segment(0,3).cast<double>(),RGps,PredGps,false);
  GenSigPoints();
}
void MagUpdate(){
  getMag(mag);
  Meas<3>(mag.normalized(),RMag,PredMag,true);
  Eigen::Vector3d angErr=x_mean.segment(9,3);
  Eigen::Quaterniond QErr;
  if(angErr.norm()>1){
    //shadow set correction
    angErr=-angErr/(angErr.squaredNorm());
  }
  QErr.w()=(1-angErr.squaredNorm())/(1+angErr.squaredNorm());
  QErr.vec()=2*angErr/(1+angErr.squaredNorm());
  inerToBody=QErr*inerToBody;//Updates inerToBody quaternion
  x_mean.segment(9,3)=Eigen::Vector3d::Zero();
  GenSigPoints();
}
void getOrientation(Eigen::Vector3d a, Eigen::Vector3d m){//gets initial organization using triad with magnetometer and gravity direction
  if(DEBUG){
    Serial.println("Getting Orientation...");
  }
  a.normalize();
  m.normalize();
  //a is treated as body down axis e.g. negative of accelerometer readings

  Eigen::Vector3d bodyNorth=(m-a*a.dot(m));
  bodyNorth.normalize();
  Eigen::Vector3d bodyEast=a.cross(bodyNorth);
  bodyEast.normalize();
  bodyNorth=bodyEast.cross(a).normalized();
  //Storing recorded [DOWN,NORTH,EAST] in body axes 
  Eigen::Matrix<double,3,3> BodyRmat;
  BodyRmat.col(0)=a;
  BodyRmat.col(1)=bodyNorth;
  BodyRmat.col(2)=bodyEast;
  //IF Down, north east, found in GetGPS();
  Eigen::Matrix<double,3,3> IFRmat;
  IFRmat.col(0)=inertialDown;
  IFRmat.col(1)=inertialNorth;
  IFRmat.col(2)=inertialEast;
  if(DEBUG){
    Serial.println("DOWN, NORTH, EAST BODY DIRECTIONS (columns)");
    printMatrix(BodyRmat);
    Serial.println("DOWN, NORTH, EAST I.F. DIRECTIONS (columns)");
    printMatrix(IFRmat);
  }
  magDir=IFRmat*magDir;
  Eigen::Vector3d magNorth=magDir-inertialDown*(inertialDown.dot(magDir));// Magnetic North Vector in inertial frame
  magNorth.normalize();
  IFRmat.col(1)=magNorth;
  IFRmat.col(2)=(inertialDown.cross(magNorth)).normalized();
  IFRmat.col(1)=IFRmat.col(2).cross(inertialDown.normalized());
  //IFRmat North vector now aligns with magnetic north tangent to ellipsoid
  //now putting Inertial->Body rotation matrix into BodyRmat
  BodyRmat=BodyRmat*IFRmat.transpose();
  magDir=BodyRmat.transpose()*m;
  //and setting Inertial to Body quaternion
  inerToBody=Eigen::Quaterniond(BodyRmat);
  if(DEBUG){
    Serial.println("magDir IF");
    printMatrix(magDir.transpose());
    Serial.println("magDir Body");
    printMatrix((inerToBody*magDir).transpose());
    Serial.print("Body reading");
    printMatrix(m.transpose());
    Serial.print("Magnetic DOWN, NORTH, EAST");
    printMatrix(IFRmat);
    Serial.println("Body->Inertial Rotation Matrix");
    printMatrix(BodyRmat);
    Serial.println("Orientation Got!");
  }
}
Eigen::Matrix<double,25,1> FlightData(){
  Eigen::Matrix<double,25,1> Dat;
  Dat.segment(0,6)=x_mean.segment(0,6);//Pos&Vel (relative to initial Position)
  Dat.segment(6,3)=inerToBody.vec(); //CHANGE, this
  Dat.segment(9,2)=Eigen::Vector2d::Zero();//CHANGE needs BMP data
  Dat.segment(11,3)=x_mean.segment(6,3);//CHANGE, this is the angular error MRP, ideally this is zero
  Dat.segment(14,3)=x_mean.segment(12,3)*180/M_PI;//Gyro Biases (in Deg/s)
  Dat.segment(17,6)=Sx.diagonal().segment(0,6);//Pos & Vel errors
  Dat(23)=((Sx.diagonal()).segment(9,3)).norm()*180/M_PI;//angular error MRP uncertainty/error, this ought to be sent to GC as a single value in deg
  Dat(24)=Time;//Time since system start in ms
  //CHANGE, include recorded body rates (before bias corrections)
  return Dat;
}
void startDataLog(){
  File dataFile=SD.open("FlightData.txt",FILE_WRITE);
  if(dataFile){
  dataFile.println();
  dataFile.println("New Flight");
  dataFile.print(initialCoords(0));
  dataFile.print(",");
  dataFile.print(initialCoords(1));
  dataFile.print(",");
  dataFile.println(initialCoords(2));
  dataFile.println("X,Y,Z,VX,VY,VZ,Modified Rodriguez Param,Pa,C,ABX,ABY,ABZ,GBX,GBY,GBZ,ErrX,ErrY,ErrZ,ErrVX,ErrVY,ErrVZ,ErrHdg,Time");
  dataFile.close();
  //all angles except quaternion angles given in deg/s & deg. pos, vel, ab given in m,m/s,m/s/s
  }else{
    Serial.println();
  }
}
void logData(){
  //logs x,y,z, vx, vy, vz, q1, q2, q3, q4, pressure recorded, temperature recorded (C), accel biases xyz, gyro biases xyz,
  //uncertainty xyz, v xyz, heading (single value)  
  File dataFile=SD.open("FlightData.txt",FILE_WRITE);
  Eigen::Matrix<double,25,1> dataToWrite=FlightData();
  int dataString=dataToWrite.rows();//CHANGE this probalby isnt needed
  if(dataFile){
    for(int i=0; i<dataString; i++){
      dataFile.print(dataToWrite(i));
      dataFile.print(",");
    }
    dataFile.println();
    dataFile.close();
    if(DEBUG){
      Serial.println("Data Write Complete");
      Serial.println("Data:");
      printMatrix(dataToWrite.transpose());
    }
  }else{
    Serial.println("Error Opening FlightData.txt");
    Serial.println("Write Failed");
  }
}
//CHANGE, need packet scheme, this will be the slave device to Grond Control and send confirmation of recieved commands, will confirm each packet is sent by sending packet and listening for packet identifier+acknowledgement code

/*
current ideas
package name is first byte
pre-launch package (initial Pos, initial heading (MRP's), system time at startup)
^IP is doubles, heading is floats, time is float
8*3+4*3+4=40 bytes

//All launch packet data will be stored at the same time, but packets sent individually

Launch Packet 1: Pos, Vel, orientation MRP's, time
all floats 4*3+4*3+4*3+4=40 bytes

Launch Packet 2: Pressure,dP/dt (1s avg), Temperature, Error in Pos, Error in Vel, Single angle error in heading
4+4+3*4+3*4+4=36 bytes

Launch Packet 3: Accelerometer Biases, Gyro Biases, Current Mode (0 UKF/1 Manual), Stage 1 LOCKOUT (0 OFF/1 ON), Stage 2 LOCKOUT (0 OFF, 1 ON)
Have I blown Charge 1? (0 NO/ 1 YES), Have I blown Charge 2? (0 NO/ 1 YES), Am I ready to blow Charge 1?, Am I ready to blow Charge 2
//CHANGE, better way to send the status
4*3+4*3+1+1+1 +1+1+1+1
31 bytes

Launch Packet 4: Accel Rates, Gyro Rates, Humidity (float)
3*4+3*4+4=16 bytes
*/



/*
double getUndulation(Eigen::Vector3d POS){
  //returns Geoid undulation in m for given pos in geodectic coords
  //convert geodectic coords to lat/lon
  if(DEBUG){
    Serial.println("Getting Geoid Undulation...");
  }
  while (true){
    while(Serial1.available()){
      gps.encode(Serial1.read());
    }
    if(DEBUG){
      Serial.println(gps.charsProcessed());
    }
    if(gps.location.isValid()){
      Serial.println("Found location");
      break;
    }
    delay(50);
  }
  Eigen::Vector3d coords(gps.location.lat(),gps.location.lng(),NAN);
  Eigen::Matrix<double,3,5> cornerPoints;// (Lat, Lon, undulation) of closest data points
  File readFile=SD.open("EGM2008 - 5'.pgm");//5 minute resolution of EGM 2008
  const uint32_t HEADER_SIZE = 108;
  const uint32_t GRID_WIDTH=4321;
  //set up corner points lat/lon here
  if(readFile){
    uint32_t row; 
    uint32_t col;
    uint32_t pos;
    for(int i=0; i<4; i++){
    row=0;//CHANGE
    col=0;//CHANGE

    pos=HEADER_SIZE + ((row*GRID_WIDTH)+col)*2;//position of byte on pgm
    if(readFile.seek(pos)){
      uint8_t hi=dataFile.read();
      uint8_t lo=dataFile.read();
      uint16_t raw=(hi<<8) | lo;
      cornerPoints(3,i)=(raw*0.003)-108.0;//conversion to m
      Serial.print("Geoid Corner ");
      Serial.print(i);
      Serial.println(" data");
      printMatrix(cornerPoints.col(i).transpose());
    }
    }
    //interpolation here
    readFile.close();
  }

  //I need to download the data to actually write the math
}
*/
void setup() {
  magDir<<49617,19075.4,-1332.7;//CHANGE XYZ geomag library
  //CHANGE magNorm checks
  magDir.normalize();
  Serial.begin(115200);
  Serial1.begin(9600);//Starting GPS line
  while(!Serial1);//waiting for GPS line to start
  Wire.begin();//Starting I2C line
  delay(1000);
  Serial.println("=======Initializing=======");
  init9150();
  if(!SD.begin(chipSelect)){
    Serial.println("SD card initialization error");
  }
  delay(500);
  //Getting our initial position and covariance
  
  while(true){
    Serial.println("Waiting for GPS...");
    while(Serial1.available()>0){
    gps.encode(Serial1.read());
    }
    if (gps.charsProcessed() > 10) {
      Serial.println("GPS online");
    }
    Serial.println(gps.charsProcessed());
    if(gps.location.isUpdated()&&gps.location.isValid()&&gps.altitude.isValid()&&gps.hdop.isValid()){
      break;
    }
    delay(50);
  }
  Eigen::Matrix<double,6,1>GPSDat=GetGps();
  initialCoords=GPSDat.segment(0,3);//setting up our launch area. transmit this to ground station in pre-launch package
  Serial.println("Got GPS");
  startDataLog();
  if(DEBUG){
  }

  //Running a calibration until we have a std_dev in angular errors below 1deg
  Eigen::Vector3d accelAngDev=Eigen::Vector3d::Zero();
  Eigen::Vector3d magAngDev=Eigen::Vector3d::Zero();
  double deltaAccDev=0;
  double deltaMagDev=0;
  Eigen::Vector3d a=Eigen::Vector3d::Zero();
  Eigen::Vector3d m=Eigen::Vector3d::Zero();
  int n=0;
  Eigen::Vector3d accelSum=Eigen::Vector3d::Zero();
  Eigen::Vector3d accelSquareSum=Eigen::Vector3d::Zero();
  Eigen::Vector3d magSum=Eigen::Vector3d::Zero();
  Eigen::Vector3d magSquareSum=Eigen::Vector3d::Zero();
  Serial.println("Beginning Calibration");

  while ((deltaAccDev>0.1&&deltaMagDev>0.1)||n<50){//repeats loop until both vectors have a cahnge in variance of <=0.1 rad after at least 50 samples
    delay(100);
    a=getINS().segment(0,3);
    getMag(m);
    n++;
    accelSum+=a;
    magSum+=m.normalized();
    accelSquareSum+=a.cwiseAbs2();
    magSquareSum+=(m.normalized()).cwiseAbs2();
    
    //finding std deviation
    deltaAccDev=(accelAngDev-(accelSquareSum-accelSum.cwiseAbs2()/n)/n).norm();
    deltaMagDev=(magAngDev-(magSquareSum-magSum.cwiseAbs2()/n)/n).norm();
    accelAngDev=(accelSquareSum-accelSum.cwiseAbs2()/n)/n;
    magAngDev=(magSquareSum-magSum.cwiseAbs2()/n)/n;
    Serial.print("Loop");
    if(DEBUG){
      Serial.println("Initial GPS read");
      printMatrix(GPSDat.cast<double>());
      Serial.print("Accelerometer Reading");
      printMatrix(a);
      Serial.print("Magnetometer Reading");
      printMatrix(m);
      Serial.println("Accelerometr Angular Deviation");
      printMatrix(accelAngDev);
      Serial.print("Magnetic Angular Deviation: ");
      printMatrix(magAngDev);
    }
  }
  x_mean=Eigen::Matrix<double,StateNum,1>::Zero();
  x_mean.segment(0,3)=Eigen::Vector3d::Zero();//position is measured as distance from starting point for UKF
  getOrientation(-accelSum/n,magSum/n);//negative on accelSum so it points downwards
  //Setting up our uncertainties
  Eigen::Matrix<double,StateNum,1> StateCov=Eigen::Matrix<double,StateNum,1>::Constant(1);
  StateCov.head(3)=GPSDat.segment(3,3).cast<double>();
  StateCov.segment(3,3)=Eigen::Vector3d::Constant(10);//velocity initialized with 1 m/s variance
  StateCov.segment(6,3)=Eigen::Vector3d::Constant(3);//accelerometer bias initialized with 0.5m/s/s variance
  StateCov.segment(9,3)=Eigen::Vector3d::Constant(0.10);//Small angle assumtion, variance in heading error is equal to sum of recorded gravity and magnetic variances
  StateCov.segment(12,3)=Eigen::Vector3d::Constant(0.5);//gyro bias initialized with 0.5 rad/s error (roughly 15deg/s)
  //StateCov.segment(15,2)=Eigen::Vector2d::Constant(0.5);//magnetometer offset for both axis intialized with 0.1rad variance (roughly 5deg)
  //StateCov(17)=1.0f;//Change this when I know what I'm doing
  //^CHANGE hopefully overwritten with future magnetometer calibration routine
  Sx=StateCov.asDiagonal();
  //Setting up our process noises
  Eigen::Matrix<double,ProcNum,1> ProcCov=Eigen::Matrix<double,ProcNum,1>::Constant(0.4);//CHANGE Flush this out later
  ProcCov.segment(0,3)=Eigen::Vector3d::Constant(0.3);//additve accelerimeter noise
  ProcCov.segment(3,3)=Eigen::Vector3d::Constant(0.1);//additive gyro noise
  //CHANGE add walk bias modeling
  Sw=ProcCov.asDiagonal();
  if(DEBUG){
    Serial.print("W0: ");
    Serial.println(W0);
    Serial.print("Wi: ");
    Serial.println(W1);
    Serial.println("initial State");
    printMatrix(x_mean.transpose());
    Serial.println("Sx matrix");
    printMatrix(Sx);
    Serial.println("Sw matrix");
    printMatrix(Sw);
  }
  GenSigPoints();
  //Some constant R matricies are set up here
  RMag=Eigen::Vector3d::Constant(0.4).asDiagonal();//change something better
  RStill=Eigen::Matrix<double,6,1>::Constant(4).asDiagonal();
  //CHANGE Send pre launch package and await ON command here
  delay(500);
  rates=getINS();
  lastMillis=millis();
}
float magtimer=0;
void loop() {
  // put your main code here, to run repeatedly:
  if(DEBUG){
    Serial.println("Loop Started");
  }
  if(DEBUG){
    Serial.println("New Sigma Points (Col Vectors)");
    printMatrix(SigPoints);
  }
  //Update Timers
  dt=double((millis()-lastMillis))/1000.0f;
  lastMillis=millis();
  timer+=dt;
  logTimer+=dt;
  Time+=dt;
  magtimer+=dt;
  //propogate the model with new dt, get rates for next propogation
  sigPointsProp();
  GenSigPoints();
  rates=getINS();
  //check timer and log data to SD card
  if(logTimer>1){
    if(DEBUG){
    Serial.println("Logging Data...");
    }
    logData();
    logTimer=0;
  }
  //Debugging stuff
  if(DEBUG){
    Serial.println("State");
    printMatrix(x_mean.transpose());
    Serial.println("New Quaternion:");
    Serial.print(inerToBody.w());
    Serial.print(", ");
    printMatrix(inerToBody.vec());
    Eigen::Matrix3d DispMat=inerToBody.toRotationMatrix().transpose();
    Serial.println("Body to Inertial Rmat");
    printMatrix(DispMat);
    Serial.println();
    Serial.println("Body Accelerations");
    Eigen::Vector3d Accelerations=rates.segment(0,3);
    Accelerations=Accelerations-x_mean.segment(6,3);
    printMatrix(Accelerations.transpose());
    Serial.println("IF Accelerations");
    Accelerations=DispMat*rates.segment(0,3)+inertialDown*Grav;
    printMatrix(Accelerations.transpose());
  }
  //Run measurement updates
  GPSUpdate();
  if(magtimer>0.03){
    MagUpdate();
    magtimer=0;
  }
  //transmission stuff


  if(DEBUG){
    Serial.println("End of Loop");
  }
  
  delay(10);//adding a small delay so we never go >100hz (MPU9150 max rate) (CHANGE later maybe)
}

/*  
  Old calibration code that might still work. Hold system still, rotate it around a bit while these checks are occuring. Uses accelerometer reading for downwards direction
    Eigen::Matrix<double,6,1> stillMeas;
    stillMeas<<initialPos,Eigen::Vector3d::Constant(0);
    Meas<6>(stillMeas,RStill,PredStill);
    GenSigPoints();
    calibtimer+=dt;
    //
    Eigen::Vector3d Down=-rates.segment(0,3);
    Meas<3>(Down.normalized(),RMag,PredDown);
    smallAngleUpdate.w()=1;
    smallAngleUpdate.vec()=State.segment(9,3)*0.5;
    inerToBody=inerToBody*smallAngleUpdate;
    State.segment(9,3)=Eigen::Vector3d::Constant(0);//resets expected angular error to 0
    inerToBody.normalize();
    */
