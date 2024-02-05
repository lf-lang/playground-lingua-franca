## Tug of War

Tug of War is a team-based game where two teams pull on opposite ends of a rope to bring the other team across a center marker. The demo involves two players per team, each characterized by a parameter. Players apply a randomly generated force within the interval 1 and a specified maximum force parameter. The sum of forces on each side moves the marker, and when it reaches one of the limits, the game ends.

## How it works

This example features a browser-based UI. The server is constructed using [Flask](https://flask.palletsprojects.com/en/3.0.x/), a Python web framework, and [Dash](https://dash.plotly.com/) components.

Every second, player forces are generated and their values are send to the server as 
a post request. The UI's gauges are updated with these values, along with the position of the mark (yellow square). When the mark reaches one of the limits (pink diamond), the label on the top of the page is updated with the result.

## Steps:

 1. Compile `TugOfWar.lf`. 
 2. Launch the UI by running the script `start.sh` under `C/src/TugOfWar/Interface`. The script will create a virtual environment and install the requirements listed in `requirements.txt`, if not there already. It then lauches Falsk server, accessible on: [http://127.0.0.1:5004](http://127.0.0.1:5004).
 3. Run the launching script under `bin` and watch the game on the UI. 

