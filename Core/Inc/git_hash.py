
import os
from git import Repo # pip3 install GitPython
from pathlib import Path

cwd = Path.cwd()
print("Current Dir:" + str(cwd))

upper_directory = str(cwd.parent) # go to the upper directory
print("Upper Dir:" + upper_directory)

h_file_path = upper_directory+"/Core/Inc/git_commit_hash.h"
print("H file Dri:" + h_file_path)

repo = Repo(upper_directory)
commit_hash = repo.git.rev_parse("HEAD")
branch_name = repo.active_branch.name

version_major = branch_name[13]
version_minor = int(branch_name[15])*10 + int(branch_name[16])

macro_def = "#define CURRENT_COMMIT_HASH "+'"'+commit_hash[0:8]+'"'

if os.path.exists(h_file_path):
    os.remove(h_file_path)

hash_file = open(h_file_path, "w")
hash_file.write(macro_def)
hash_file.write("\r\n")
hash_file.write("#define VERSION_MAJOR "+str(version_major))
hash_file.write("\r\n")
hash_file.write("#define VERSION_MINOR "+str(version_minor))
