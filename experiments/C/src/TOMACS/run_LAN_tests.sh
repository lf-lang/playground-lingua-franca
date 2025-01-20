#!/bin/bash

folders=("LAN_ZeroDelayCycle" "LAN_MicrostepDelayCycle" "LAN_Consistency" "LAN_ConsistencyMicrostepDelay" "LAN_Feedback" "LAN_FeedbackMicrostepDelay")
delays=(1 2 3 5 10 100)

for i in {1..10}; do
    for folder in "${folders[@]}"; do
		if [[ -d "$folder" ]]; then
			echo "Compile .lf files in $folder:"
			for file in "$folder"/*.lf; do
				if [[ -f "$file" ]]; then
					filename_with_ext="${file##*/}"
					filename="${filename_with_ext%.lf}"
					echo "File name is $filename."
					if [ -f "../../bin/$filename" ]; then
						echo "File $filename exists in the directory 'bin'."
					else
						echo "lfc $file."
						$lfc "$file"
					fi

					if [[ $? -eq 0 ]]; then
						mkdir -p "./Results/LAN_Results/$folder/$filename/$i"

						cp "../../bin/$filename" "./Results/LAN_Results/$folder/$filename/$i"
						echo "Compiled $file successfully and moved to Results/$folder/$filename/$i"
						pushd ./Results/LAN_Results/$folder/$filename/$i
						# ./$filename
						sleep 5
						popd
						echo "Ran ./Results/LAN_Results/$folder/$filename/$i."
					else
						echo "Failed to compile $file."
					fi
				else
					echo "No .lf files found in $folder."
				fi
			done
		else
			echo "Directory $folder does not exist."
		fi
	done
done

echo "LAN tests Done"
