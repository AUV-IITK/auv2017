TaskHandler for moving from one line to another.
Use Orange.action for calling this node to move from one line to another.
Send it a boolean to start and a bool telling whether it is already on a line or not.

TODO :
add comunication with forwardServer "forward" node

write an orange server. Goal is bool , if goal is true then there is a line below. feedback no significance. Result is true if all went well. or else some error.

write an orange client

#Write a LinetoLineTaskHandler which is a actionlib server for the master node. It takes bool to start and sends some feedback. Then sends result as boolean when done.
#make this LinetoLineTaskHandler deal with forward (from the motionlibrary)

Add a wiki on how to add dynamic reconfig to any package
Add a wiki on how to add actionlib to any package
Add a wiki explaning how all our code works.
Delete unnecessary issues. And move the documentation from issues to wiki
