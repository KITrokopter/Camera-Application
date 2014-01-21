#!/usr/bin/env bats

@test "initialize_camera service existance" {
    run rosservice list
    [ $status -eq 0 ]
    echo "$output" | grep -q "^/initialize_camera$"
}

@test "querying initialize_camera with empty parameters returns no error" {
    run rosservice call /initialize_camera '{header: auto, hsvColorRanges: [], quadCopterIds: []}'
    [ $status -eq 0 ]
    [ $output = "error: 0" ]
}

@test "querying initialize_camera a second time returns an error" {
    run rosservice call /initialize_camera '{header: auto, hsvColorRanges: [], quadCopterIds: []}'
    [ $status -eq 0 ]
    [ $output = "error: 1" ]
}

# vi: ft=sh
