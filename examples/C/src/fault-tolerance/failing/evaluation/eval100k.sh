# Change ../cleanlf/bin/lfc-dev to lfc path.

# Compile all 9 Lingua Franca programs
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalApproach05.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalNoAdvance05.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalWorstCase05.lf

../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalApproach1.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalNoAdvance1.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalWorstCase1.lf

../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalApproach2.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalNoAdvance2.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalWorstCase2.lf

../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalApproach5.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalNoAdvance5.lf
../cleanlf/bin/lfc-dev examples/C/src/CheckpointRestore/failing/evaluation/EvalWorstCase5.lf

# Run the binaries and save their output

#### Ubuntu
/usr/bin/time -v ./examples/C/bin/EvalApproach05 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail0.txt 2>&1
echo "✅ Finished EvalApproach05"

/usr/bin/time -v ./examples/C/bin/EvalNoAdvance05 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail0.txt 2>&1
echo "✅ Finished EvalNoAdvance05"

/usr/bin/time -v ./examples/C/bin/EvalWorstCase05 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail0.txt 2>&1
echo "✅ Finished EvalWorstCase05"


/usr/bin/time -v ./examples/C/bin/EvalApproach1 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail1.txt 2>&1
echo "✅ Finished EvalApproach1"

/usr/bin/time -v ./examples/C/bin/EvalNoAdvance1 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail1.txt 2>&1
echo "✅ Finished EvalNoAdvance1"

/usr/bin/time -v ./examples/C/bin/EvalWorstCase1 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail1.txt 2>&1
echo "✅ Finished EvalWorstCase1"


/usr/bin/time -v ./examples/C/bin/EvalApproach2 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail2.txt 2>&1
echo "✅ Finished EvalApproach2"

/usr/bin/time -v ./examples/C/bin/EvalNoAdvance2 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail2.txt 2>&1
echo "✅ Finished EvalNoAdvance2"

/usr/bin/time -v ./examples/C/bin/EvalWorstCase2 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail2.txt 2>&1
echo "✅ Finished EvalWorstCase2"


/usr/bin/time -v ./examples/C/bin/EvalApproach5 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail5.txt 2>&1
echo "✅ Finished EvalApproach5"

/usr/bin/time -v ./examples/C/bin/EvalNoAdvance5 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail5.txt 2>&1
echo "✅ Finished EvalNoAdvance5"

/usr/bin/time -v ./examples/C/bin/EvalWorstCase5 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail5.txt 2>&1
echo "✅ Finished EvalWorstCase5"


################ MAC environment

gtime -v ./examples/C/bin/EvalApproach05 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail0.txt 2>&1
echo "✅ Finished EvalApproach05"

gtime -v ./examples/C/bin/EvalNoAdvance05 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail0.txt 2>&1
echo "✅ Finished EvalNoAdvance05"

gtime -v ./examples/C/bin/EvalWorstCase05 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail0.txt 2>&1
echo "✅ Finished EvalWorstCase05"


gtime -v ./examples/C/bin/EvalApproach1 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail1.txt 2>&1
echo "✅ Finished EvalApproach1"

gtime -v ./examples/C/bin/EvalNoAdvance1 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail1.txt 2>&1
echo "✅ Finished EvalNoAdvance1"

gtime -v ./examples/C/bin/EvalWorstCase1 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail1.txt 2>&1
echo "✅ Finished EvalWorstCase1"


gtime -v ./examples/C/bin/EvalApproach2 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail2.txt 2>&1
echo "✅ Finished EvalApproach2"

gtime -v ./examples/C/bin/EvalNoAdvance2 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail2.txt 2>&1
echo "✅ Finished EvalNoAdvance2"

gtime -v ./examples/C/bin/EvalWorstCase2 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail2.txt 2>&1
echo "✅ Finished EvalWorstCase2"


gtime -v ./examples/C/bin/EvalApproach5 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalApproach_fail5.txt 2>&1
echo "✅ Finished EvalApproach5"

gtime -v ./examples/C/bin/EvalNoAdvance5 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalNoAdvance_fail5.txt 2>&1
echo "✅ Finished EvalNoAdvance5"

gtime -v ./examples/C/bin/EvalWorstCase5 > examples/C/src/CheckpointRestore/failing/evaluation/res100k/EvalWorstCase_fail5.txt 2>&1
echo "✅ Finished EvalWorstCase5"




echo "🎉 All runs completed!"
