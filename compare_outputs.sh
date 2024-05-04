echo "Comparing ground truth outputs to new processor" | tee -a debug.log

NEW_IMP_PATH="/user/stud/spring24/df2817/csee4824-final-project"

REF_DIR="$NEW_IMP_PATH/p3-out"
OUT_DIR="$NEW_IMP_PATH/output"

cd $NEW_IMP_PATH
# source setup-paths.sh
# make nuke 

#!/bin/bash

# Check if a filename was provided
if [ -z "$1" ]
then
    # This only runs *.s files. How could you add *.c files?
    for source_file in programs/*.s programs/*.c; do
        if [ "$source_file" = "programs/crt.s" ]
        then
            continue
        fi
        program=$(echo "$source_file" | cut -d '.' -f1 | cut -d '/' -f 2)
        
        echo "Running $program" | tee -a debug.log
        make $program.out >> debug.log 2>&1 

        echo "Comparing writeback output for $program" | tee -a debug.log
        diff --color -u "$REF_DIR/$program.wb" "$OUT_DIR/$program.wb" >> debug.log 2>&1 
        WRITEBACK=$?
        echo "Writeback result: $WRITEBACK" | tee -a debug.log

        echo "Comparing memory output for $program" | tee -a debug.log
        diff --color -u <(grep "^@@@" "$REF_DIR/$program.out") <(grep "^@@@" "$OUT_DIR/$program.out") >> debug.log 2>&1 
        MEM=$?

        echo "Printing Passed or Failed" | tee -a debug.log
        if (($WRITEBACK != 0 || $MEM != 0))
        then
            echo "Failed" | tee -a debug.log
        else 
            echo "Passed" | tee -a debug.log
        fi
    done
else
    # Check if the file exists
    if [ ! -f "$1" ]
    then
        echo "File does not exist."
        exit 1
    fi

    program=$(echo "$1" | cut -d '.' -f1 | cut -d '/' -f 2)
    echo "Running $program" | tee -a debug.log
    
    make $program.out #>> debug.log 2>&1 

    echo "Comparing writeback output for $program" #| tee -a debug.log
    diff --color -u "$REF_DIR/$program.wb" "$OUT_DIR/$program.wb" # >> debug.log 2>&1 
    WRITEBACK=$?
    echo "Writeback result: $WRITEBACK" #| tee -a debug.log

    echo "Comparing memory output for $program" #| tee -a debug.log
    diff --color -u <(grep "^@@@" "$REF_DIR/$program.out") <(grep "^@@@" "$OUT_DIR/$program.out") #| tee -a debug.log
    MEM=$?
    echo "Memory result: $MEM" #| tee -a debug.log

    echo "Printing Passed or Failed" #| tee -a debug.log
    if (($WRITEBACK != 0 || $MEM != 0))
    then
        echo "Failed" #| tee -a debug.log
    else 
        echo "Passed" #| tee -a debug.log
    fi
fi