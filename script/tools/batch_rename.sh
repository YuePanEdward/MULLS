#!/bin/sh

target_folder=xxxx/xxxx
file_format=.csv
file_extension=_project;

cd ${target_folder}

# rename 's[s means substitution] / [part to rename] / [part to subsitute] /' [files for processing]

# For more than 10000 datas
rename 's/_/_0000/' ${file_head}[0-9]${file_format}
rename 's/_/_000/' ${file_head}[0-9][0-9]${file_format}
rename 's/_/_00/' ${file_head}[0-9][0-9][0-9]${file_format}
rename 's/_/_0/' ${file_head}[0-9][0-9][0-9][0-9]${file_format}

# For more than 1000 datas
# rename 's/_/_000/' [0-9]${file_extension}${file_format}
# rename 's/_/_00/' [0-9][0-9]${file_extension}${file_format}
# rename 's/_/_0/' [0-9][0-9][0-9]${file_extension}${file_format}

# delete all the spaces
# rename 's/ /_/g' *

# create file_anme_list
#cd ..
#ls ${target_folder} >> ${file_name_list_name}