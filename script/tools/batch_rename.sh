#!/bin/sh

target_folder=xxxx/xxxx
file_format=.pcd
file_head= ;
file_extension= ;

# Brief
# the original filename is:  ${file_head}1${file_extension}${file_format}
# you want to convert the filename to something like: ${file_head}00001${file_extension}${file_format}
# Then you can use this shell file and do the renaming (add the zeros) in batch

cd ${target_folder} 

# rename 's[s means substitution] / [part to rename] / [part to subsitute] /' [files for processing]

# Depends on the number of your files
# if the number is larger than 10000
# run (or comment)
rename 's//0000/' ${file_head}[0-9]${file_extension}${file_format}
rename 's//000/' ${file_head}[0-9][0-9]${file_extension}${file_format}
rename 's//00/' ${file_head}[0-9][0-9][0-9]${file_extension}${file_format}
rename 's//0/' ${file_head}[0-9][0-9][0-9][0-9]${file_extension}${file_format}

# if the number is larger than 1000 but smaller than 10000
# run (or comment)
# rename 's//000/' [0-9]${file_format}
# rename 's//00/' [0-9][0-9]${file_format}
# rename 's//0/' [0-9][0-9][0-9]${file_format}

# if the number is larger than 100 but smaller than 1000
# run (or comment)
# rename 's//00/' [0-9]${file_format}
# rename 's//0/' [0-9][0-9]${file_format}

# if the number is larger than 10 but smaller than 100
# run (or comment)
# rename 's//0/' [0-9]${file_format}

# delete all the spaces
# rename 's/ /_/g' *

# create file_anme_list
#cd ..
#ls ${target_folder} >> ${file_name_list_name}