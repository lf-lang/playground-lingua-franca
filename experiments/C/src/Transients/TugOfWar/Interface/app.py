import pandas as pd
import dash
from dash import dcc, html, Input, Output
from dash.exceptions import PreventUpdate
import plotly.express as px
from flask import Flask, request
import json
import app_helper as ap
import dash_bootstrap_components as dbc
import dash_daq as daq


player_force = dict({
    'players': ['Team_A_Player_0', 'Team_A_Player_1', 'Team_B_Player_0', 'Team_B_Player_1'],
    'forces': [0, 0, 0, 0],
    'score': 'Advantage: None',
    'rope_mark': 20
})

print(player_force)

limit1 = 5
limit2 = 35
rope_range = 40


# Create Flask server
server = Flask(__name__)

# Create Dash app
app = dash.Dash(__name__, server=server,external_stylesheets=[dbc.themes.BOOTSTRAP])

df = ap.build_rope_dataframe(limit1, limit2, player_force['rope_mark'])

# Define layout
app.layout = html.Div([
    dbc.Card(
        dbc.CardBody([

            # Row 1: title
            dbc.Row([
                dbc.Col([
                    html.Div(
                        dbc.CardImg(src='assets/lf.png',
                            style={'backgroundColor': '#013163', 'width': '40%', 'height': '40%'},
                            id='logo'
                        )
                    , style={'align': 'center'})
                ], align='center',  width=2),
                dbc.Col([
                    html.Div(ap.drawTextCard("TUG OF WAR"), 
                        id='title'),
                ], width=10),
            ], style={'backgroundColor': '#013163'}, align='center'), 
            html.Br(),

            ### Row 2: Score
            dbc.Row([
                dbc.Col([
                    html.Div(
                        dbc.Card([
                            dbc.CardBody([
                                html.Div([
                                    dcc.Input(id='score', value=player_force['score'], type="text", style={'fontSize': '20px'})
                                ], style={'textAlign': 'center'})
                            ])
                        ])
                    ),
                ], width=12),
            ], align='center'), 
            html.Br(),

            ### Row 3: Rope
            dbc.Row([
                dbc.Col([
                    html.Div(
                        dbc.Card(
                            dbc.CardBody([
                                dcc.Graph(
                                    id='rope',
                                    figure=px.scatter(df, 
                                        x="x", 
                                        y="y", 
                                        color="val", 
                                        symbol='val', 
                                        size='val',
                                        range_x=[1,rope_range],
                                        range_y=[0, 0]
                                    ).update_layout(
                                        template='plotly_dark',
                                        plot_bgcolor= 'rgba(0, 0, 0, 0)',
                                        paper_bgcolor= 'rgba(0, 0, 0, 0)',
                                        xaxis={'visible':False},
                                        coloraxis={'showscale':False}
                                    ),
                                    config={
                                        'displayModeBar': False
                                    }
                                    
                                ) 
                            ])
                        ),
                    )
                ], width=12),
            ], align='center'), 
            html.Br(),

            ### Row 4: Gauges
            dbc.Row([
                ## Gauge 1
                dbc.Col([
                    html.Div(
                        dbc.Card([
                            dbc.CardBody([
                                html.Div([
                                    daq.Gauge(
                                        value=player_force['forces'][0],
                                        label=player_force['players'][0],
                                        max=20,
                                        min=0, id='gauge1'
                                    )
                                ], style={'color': 'dark', 'margin': '2', 'textAlign': 'center'})
                            ])
                        ])
                    ),
                ], width=3),
                
                ## Gauge 2
                dbc.Col([
                    html.Div(
                        dbc.Card([
                            dbc.CardBody([
                                html.Div([
                                    daq.Gauge(
                                        value=player_force['forces'][1],
                                        label=player_force['players'][1],
                                        max=20,
                                        min=0, id='gauge2'
                                    )
                                ], style={'color': 'dark', 'margin': '2', 'textAlign': 'center'})
                            ])
                        ])
                    ),
                ], width=3),

                ## Gauge 3
                dbc.Col([
                    html.Div(
                        dbc.Card([
                            dbc.CardBody([
                                html.Div([
                                    daq.Gauge(
                                        value=player_force['forces'][2],
                                        label=player_force['players'][2],
                                        max=20,
                                        min=0, id='gauge3'
                                    )
                                ], style={'color': 'dark', 'margin': '2', 'textAlign': 'center'})
                            ])
                        ])
                    ),
                ], width=3),
                
                ## Gauge 4
                dbc.Col([
                    html.Div(
                        dbc.Card([
                            dbc.CardBody([
                                html.Div([
                                    daq.Gauge(
                                        value=player_force['forces'][3],
                                        label=player_force['players'][3],
                                        max=20,
                                        min=0, id='gauge4'
                                    )
                                ], style={'color': 'dark', 'margin': '2', 'textAlign': 'center'})
                            ])
                        ])
                    ),
                ], width=3),
            ],align='center'),
            
        ])
    ), dcc.Interval(id='interval-component', interval=1000, n_intervals=0)
])

@app.callback([
        Output('gauge1', 'value'),
        Output('gauge2', 'value'),
        Output('gauge3', 'value'),
        Output('gauge4', 'value'),
        Output('rope', 'figure'),
        Output('score', 'value')
    ],
    Input('interval-component', 'n_intervals')
)
def update_layout(n):
    global player_force
    global rope_range
    global limit1
    global limit2
    df = ap.build_rope_dataframe(limit1, limit2, player_force['rope_mark'])
    fig = px.scatter(df,  
            x="x", 
            y="y", 
            color="val", 
            symbol='val', 
            size='val',
            range_x=[1,rope_range],
            range_y=[0, 0]
        ).update_layout(
            template='plotly_dark',
            plot_bgcolor= 'rgba(0, 0, 0, 0)',
            paper_bgcolor= 'rgba(0, 0, 0, 0)',
            xaxis={'visible':False},
            coloraxis={'showscale':False}
        )
    return player_force['forces'][0], \
        player_force['forces'][1], \
        player_force['forces'][2], \
        player_force['forces'][3], \
        fig, \
        player_force['score']

@server.route('/update_force', methods=['POST'])
def update_force():
    global player_force
    try:
        data = json.loads(request.data)
        player_force = data[0]
        print(player_force)
        # player_force['players'] = data['players']
        # player_force['rope_mark'] = int(data['rope_mark'])
        return 'Force values updated successfully'
    except Exception as e:
        return str(e), 400

if __name__ == '__main__':
    server.run(port=5004)
