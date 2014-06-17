#! /usr/bin/env python

import rosservice

servList = rosservice.get_service_list()
maestorServs = []
serviceArgs = dict()

for serv in servList:
	if "/get_loggers" not in serv and "/set_logger_level" not in serv:
		maestorServs.append(serv)


for serv in maestorServs:
	serviceArgs[serv] = rosservice.get_service_args(serv)

f = open("/opt/ros/fuerte/stacks/maestor/console/maestorapi.py", 'w')

#Begin writing the console dynamically. WOO

f.write("#! /usr/bin/env python\n")
f.write("import rlcompleter\n")
f.write("import readline\n")
f.write("import rosservice\n\n")
f.write("readline.parse_and_bind(\"tab: complete\")\n\n")
f.write("def ls():\n")
f.write("    return " + str(maestorServs) + "\n\n")


for serv in serviceArgs:
	f.write("def " + serv.replace("/", "") + " (" + serviceArgs[serv].replace(" ", ", ") + "):\n")
	f.write("    return rosservice.call_service(\"" + serv + "\", [" + serviceArgs[serv].replace(" ", ", ") + "])[1]\n\n")
