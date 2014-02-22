rostopic pub -1 /CalibrateCamera camera_application/CalibrateCamera "{
header: auto,
ID: 0,
imageAmount: 15,
imageDelay: 500,
boardWidth: 7,
boardHeight: 7,
boardRectangleWidth: 3,
boardRectangleHeight: 3
}"
