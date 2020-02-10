import os
import ipdb

GoalPath = "/home/motion/Desktop/Multi-Contact-Config-Sampler/res/UnevenContact1" # Needs to be updated at each run

CWD = os.getcwd()
FolderList = next(os.walk(CWD))[1]
# ipdb.set_trace()
for Folder in FolderList:
    ToFilePath = CWD + '/' + Folder + "/To.txt"
    if os.path.isfile(ToFilePath):
        # Then we need to open To.txt file
         with open(ToFilePath, "r") as fp:
             GoalIndex = fp.readline()
             GoalIndex = GoalIndex.replace('\n','')
             cmd = "cd "
             cmd+= CWD + '/' + Folder
             cmd+= " && cp ContactStatus.txt " + GoalPath + "/" + GoalIndex
             cmd+= " && cp InitConfig.config " + GoalPath + "/" + GoalIndex
             os.system(cmd)  # returns the exit code in unix
    else:
        print ("To File does not exist in" + CWD + '/' + Folder)
