#!/usr/bin/env bats

@test "picture_sending_activation topic existance" {
    run rostopic list
    [ $status -eq 0 ]
    echo "$output" | grep -q "^/picture_sending_activation$"
}

@test "sending a picture_sending_activation message (42 to 1)" {
    run rostopic pub -1 /picture_sending_activation camera_application/PictureSendingActivation '{header: auto, ID: 42, active: true}'
    [ $status -eq 0 ]
}

# vi: ft=sh
