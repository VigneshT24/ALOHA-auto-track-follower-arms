# Source and licensing notes

This repository combines project code with copied Interbotix source trees.
Upstream sources, revisions and local changes are listed in
`reference/upstream_versions.txt`. Original license files and copyright notices
remain in their source directories. Those components retain their own licenses;
this repository does not replace them with a single blanket license.

The perception package currently declares “License to be chosen by project
maintainers”. Its license needs to be resolved before describing the entire
project as licensed for reuse. Package metadata in other project directories
does not settle the perception package's license.

`models/yolov8m.pt` is a third-party Ultralytics YOLO model, not a model trained
by this project. Its redistribution and use are subject to the applicable
Ultralytics terms. No additional rights to the model are granted here.

Saved camera calibration, camera serial numbers and base separation describe
the original lab. They are included as configuration examples, not universal
robot calibration values.
