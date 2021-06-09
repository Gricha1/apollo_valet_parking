# Multiple instances of Apollo 6.0


* git clone the repository.

* you will need 3 files in each Apollo instance and to change the name of the container for each one:

1. docker/scripts/dev_start_multiuser.sh
1. docker/scripts/dev_into_multiuser.sh
1. bootstrap_multiuser.sh


* change the number of Apollo instance in ./docker/scripts/dev_start_multiuser.sh to change the name of the containers and the dreamview port:
```txt
INSTANCE_NUM=2
```
* change the number of Apollo instance in ./docker/scripts/dev_into_multiuser.sh

```txt
INSTANCE_NUM=2

```
* start Apollo using the script ./scripts/bootstrap_multiuser.sh



