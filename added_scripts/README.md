I tried to make this file run by 
./bazel-bin/added_scripts/publishing_obstacles 
but it's not working because when apollo builds it doesn't see it, so instead I put the file in:
/apollo/modules/tools/perception/
so now when it builds it sees it anyway.


note:
you can always clone this repository, just add to it the maps, because I haven't push them to the git repository.
