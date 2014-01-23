#!/usr/bin/env bats

@test "initializing node 42" {
    run rosservice call /initialize_camera '{header: auto, ID: 42, hsvColorRanges: [], quadCopterIds: []}'
    [ $status -eq 0 ]
    [ $output = "error: 0" ]
}

@test "picture_sending_activation topic existance" {
    run rostopic list
    [ $status -eq 0 ]
    echo "$output" | grep -q "^/picture_sending_activation$"
}

@test "sending a picture_sending_activation message (42 to 1)" {
    run rostopic pub -1 /picture_sending_activation camera_application/PictureSendingActivation '{header: auto, ID: 42, active: true}'
    [ $status -eq 0 ]
}

@test "sending a picture_sending_activation message (41 to 0): Should be ignored" {
    run rostopic pub -1 /picture_sending_activation camera_application/PictureSendingActivation '{header: auto, ID: 41, active: false}'
    [ $status -eq 0 ]
}

@test "sending a picture_sending_activation message (42 to 0)" {
    run rostopic pub -1 /picture_sending_activation camera_application/PictureSendingActivation '{header: auto, ID: 42, active: false}'
    [ $status -eq 0 ]
}

# vi: ft=sh
