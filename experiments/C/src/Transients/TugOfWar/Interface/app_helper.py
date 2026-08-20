from dash import dcc, html
import dash_bootstrap_components as dbc
import plotly.express as px
import dash_daq as daq
import pandas as pd
import plotly.graph_objects as go

def drawFigureCard(l1, l2, m):
    df = build_rope_dataframe(l1, l2, m)
    return dbc.Card(
            dbc.CardBody([
                dcc.Graph(
                    figure=px.scatter(df, 
                        x="x", 
                        y="y", 
                        color="val", 
                        symbol='val', 
                        size='val',
                        range_x=[0,40],
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
        )

# Title field
def drawTextCard(tt):
    return dbc.Card(
        dbc.CardBody([
            html.Div([
                html.H1(tt, style={'color': 'white'}),
            ], style={'textAlign': 'center'})
        ], style={'backgroundColor': '#013163'})
    )

# Build the dataframe for the heatmap
def build_rope_dataframe(limit1, limit2, mark):
    # Create a DataFrame of zeros
    # Create lists for 'x', 'y', and 'val'
    x_values = list(range(1, 39))
    y_values = [0]
    val_values = [0] * len(x_values)
    data = {'x': [], 'y': [], 'val': []}
    for y in y_values:
        data['x'].extend(x_values)
        data['y'].extend([y] * len(x_values))
        data['val'].extend(val_values)
    df = pd.DataFrame(data)
    # Render the limits
    df.loc[(df['x'] == limit1) & (df['y'] == 0), 'val'] = 50
    df.loc[(df['x'] == limit2) & (df['y'] == 0), 'val'] = 50
    # # Render the mark
    df.loc[(df['x'] == mark) & (df['y'] == 0), 'val'] = 100
    return df
