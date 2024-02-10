#!/bin/bash
if [ ! -f ".lf_env/bin/activate" ] 
then
    # Create a virtual environment
    virtualenv .venv

    # Activate the virtual environment
    source .venv/bin/activate

    # Install Flask
    pip install -r requirements.txt
else 
    # Activate the virtual environment
    source .venv/bin/activate
fi

# Start the Flask and Dash application
python3 app.py