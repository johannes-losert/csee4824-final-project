echo "Generating ground truth outputs from original processor" | tee -a debug.log

REF_IMP_PATH="/user/stud/spring24/df2817/Proj3-Reference"
NEW_IMP_PATH="/user/stud/spring24/df2817/csee4824-final-project"

REF_OUT="p3-out"

cd $REF_IMP_PATH

source setup-paths.sh
mkdir "$NEW_IMP_PATH/$REF_OUT"

# Copy our programs to the reference implementation
rm -r "$REF_IMP_PATH/programs/"
cp -r "$NEW_IMP_PATH/programs/" "$REF_IMP_PATH/programs/"

# This runs both *.s and *.c files
for source_file in programs/*.s programs/*.c; do
    if [ "$source_file" = "programs/crt.s" ]
    then
        continue
    fi
    program=$(echo "$source_file" | cut -d '.' -f1 | cut -d '/' -f 2)
    
    echo "Running $program" | tee -a debug.log

    make $program.out >> debug.log 2>&1 

    # Copy the generated file to our output directory
    cp output/$program.out "$NEW_IMP_PATH/$REF_OUT/$program.out"

done