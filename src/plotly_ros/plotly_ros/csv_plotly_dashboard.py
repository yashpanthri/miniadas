#!/usr/bin/env python3

'''
CARLA and PID Data Dashboard from CSV files

Dashboard visualizations for each dataset:
1. X Position vs. Time
2. Y Position vs. Time
3. Y Position vs. X position
4. GNSS Longitude/Latitude vs. Time
5. GNSS Longitude vs. Latitude
6. IMU Acceleration (X, Y, Z) vs. Time
7. Speed vs. Time

Features:
- Side-by-side comparison of PID and CARLA data
- Overlay of CARLA data on PID graphs for direct comparison
- Interactive plots with hover information
'''

import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots # for subplots
from dash import Dash, dcc, html, Input, Output # for dash app
import os # for file path

# --- Data Loading and Preprocessing ---

def preprocess_data(filepath):
    """Loads and preprocesses data from a CSV file."""
    try:
        df = pd.read_csv(filepath)
        # Convert timestamp to seconds and set as index
        df['time_sec'] = df['timestamp_ns'] / 1e9
        # Interpolate missing values for better data quality
        df = df.interpolate(method='linear', limit_direction='both')
        # Drop rows where essential data is still missing (especially at the beginning)
        df.dropna(subset=['odom.pos.x', 'speed', 'gnss.latitude', 'imu.linear.x'], inplace=True)
        
        return df
    except FileNotFoundError:
        print(f"Warning: File not found at {filepath}. Creating an empty DataFrame.")
        return pd.DataFrame(columns=[
            'time_sec', 'odom.pos.x', 'odom.pos.y', 'gnss.longitude',
            'gnss.latitude', 'imu.linear.x', 'imu.linear.y', 'imu.linear.z', 'speed'
        ])

def align_datasets_by_position(carla_df, pid_df):
    """Align datasets by finding the PID timestamp that corresponds to the first CARLA X position after movement starts."""
    if carla_df.empty or pid_df.empty:
        print("Warning: One or both datasets are empty. Cannot perform alignment.")
        return carla_df, pid_df
    
    print("Performing position-based time alignment using movement start...")
    
    # Find the start of movement in CARLA data (when X acceleration becomes non-zero)
    # Use a small threshold to detect movement
    acceleration_threshold = 0.01  # m/s²
    
    # Find first non-zero X acceleration
    movement_start_idx = None
    for i in range(len(carla_df)):
        if abs(carla_df['imu.linear.x'].iloc[i]) > acceleration_threshold:
            movement_start_idx = i
            break
    
    if movement_start_idx is None:
        print("Warning: No movement detected in CARLA data. Using first position.")
        movement_start_idx = 0
    
    # Get the CARLA position at movement start
    carla_start_x = carla_df['odom.pos.x'].iloc[movement_start_idx]
    carla_start_time = carla_df['time_sec'].iloc[movement_start_idx]
    carla_start_accel = carla_df['imu.linear.x'].iloc[movement_start_idx]
    
    print(f"CARLA movement start: X={carla_start_x:.2f}, Time={carla_start_time:.2f}, Accel={carla_start_accel:.3f} m/s²")
    
    # Truncate CARLA data to start from movement
    carla_df = carla_df.iloc[movement_start_idx:].reset_index(drop=True)
    print(f"CARLA data truncated: now starts from index {movement_start_idx}, {len(carla_df)} points remaining")
    
    # Find PID positions that are same as or just before the CARLA movement start position
    # First, find exact matches
    exact_matches = pid_df[pid_df['odom.pos.x'] == carla_start_x]
    
    if not exact_matches.empty:
        # Use the first exact match
        pid_match_idx = exact_matches.index[0]
        pid_match_x = exact_matches['odom.pos.x'].iloc[0]
        pid_match_time = exact_matches['time_sec'].iloc[0]
        match_type = "exact"
    else:
        # Find positions just before the CARLA movement start position
        positions_before = pid_df[pid_df['odom.pos.x'] <= carla_start_x]
        
        if not positions_before.empty:
            # Use the closest position that is <= CARLA movement start X
            pid_match_idx = positions_before['odom.pos.x'].idxmax()
            pid_match_x = pid_df.loc[pid_match_idx, 'odom.pos.x']
            pid_match_time = pid_df.loc[pid_match_idx, 'time_sec']
            match_type = "before"
        else:
            # If no positions before, use the first PID position
            pid_match_idx = pid_df.index[0]
            pid_match_x = pid_df['odom.pos.x'].iloc[0]
            pid_match_time = pid_df['time_sec'].iloc[0]
            match_type = "first"
    
    print(f"PID match: X={pid_match_x:.2f}, Time={pid_match_time:.2f} ({match_type} match)")
    
    # Calculate time offset to align at the reference position
    time_offset = carla_start_time - pid_match_time
    print(f"Time offset to align: {time_offset:.2f}s")
    
    # Apply the time offset to PID data
    pid_df['time_sec'] = pid_df['time_sec'] + time_offset
    
    # Now shift both datasets to start from 0
    min_time = min(carla_df['time_sec'].min(), pid_df['time_sec'].min())
    carla_df['time_sec'] = carla_df['time_sec'] - min_time
    pid_df['time_sec'] = pid_df['time_sec'] - min_time
    
    print(f"Final alignment: CARLA starts at {carla_df['time_sec'].min():.2f}s, PID starts at {pid_df['time_sec'].min():.2f}s")
    
    return carla_df, pid_df

# Load the datasets from CSV folder
csv_folder = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV'
carla_df = preprocess_data(os.path.join(csv_folder, 'carla_merged_data.csv'))
pid_df = preprocess_data(os.path.join(csv_folder, 'pid_merged.csv'))

# Align datasets by position
carla_df, pid_df = align_datasets_by_position(carla_df, pid_df)

print(f"CARLA data loaded: {len(carla_df)} rows")
print(f"PID data loaded: {len(pid_df)} rows")

# Calculate time ranges and duration for display
def get_time_info(df, name):
    if df.empty:
        return f"{name}: No data"
    duration = df['time_sec'].max() - df['time_sec'].min()
    return f"{name}: {len(df)} points, {duration:.1f}s duration ({df['time_sec'].min():.1f}s to {df['time_sec'].max():.1f}s)"

carla_info = get_time_info(carla_df, "CARLA Dataset")
pid_info = get_time_info(pid_df, "PID Dataset")

# --- Dash App Initialization ---

app = Dash(__name__)

app.layout = html.Div([
    html.H1("CARLA and PID Data Comparison Dashboard", style={'textAlign': 'center'}),

    # Dataset info
    html.Div([
        html.H3("Dataset Information", style={'textAlign': 'center'}),
        html.P(carla_info, style={'textAlign': 'center'}),
        html.P(pid_info, style={'textAlign': 'center'}),
        html.P("Note: Times have been aligned to start from 0s for comparison", 
               style={'textAlign': 'center', 'fontStyle': 'italic', 'color': 'gray'}),
    ], style={'marginBottom': '20px'}),

    # --- Comparison Graphs ---
    html.H2("Data Comparison Visualizations", style={'textAlign': 'center', 'marginTop': '40px'}),

    # X Position vs. Time
    dcc.Graph(id='odom-xt-comparison'),

    # Y Position vs. Time
    dcc.Graph(id='odom-yt-comparison'),

    # X vs. Y Position
    dcc.Graph(id='odom-xy-comparison'),

    # GNSS Longitude/Latitude vs. Time
    dcc.Graph(id='gnss-lonlat-t-comparison'),

    # GNSS Longitude vs. Latitude
    dcc.Graph(id='gnss-lonlat-comparison'),

    # IMU Acceleration vs. Time
    dcc.Graph(id='imu-accel-comparison'),

    # Speed vs. Time
    dcc.Graph(id='speed-comparison'),

], style={'width': '90%', 'margin': 'auto'})


# --- Callbacks to Populate Comparison Graphs ---

@app.callback(
    Output('odom-xt-comparison', 'figure'),
    [Input('odom-xt-comparison', 'id')]
)
def update_odom_xt_comparison(_):
    fig = go.Figure()
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['odom.pos.x'], 
            mode='lines', 
            name='CARLA X Position',
            line=dict(color='blue', width=2),
            hovertemplate='Time: %{x:.2f}s<br>X: %{y:.2f}m<extra></extra>'
        ))
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['odom.pos.x'], 
            mode='lines', 
            name='PID X Position',
            line=dict(color='red', width=2),
            hovertemplate='Time: %{x:.2f}s<br>X: %{y:.2f}m<extra></extra>'
        ))
    
    fig.update_layout(
        title='X Position vs. Time Comparison',
        xaxis_title='Time (s)',
        yaxis_title='X Position (m)',
        hovermode='x unified',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

@app.callback(
    Output('odom-yt-comparison', 'figure'),
    [Input('odom-yt-comparison', 'id')]
)
def update_odom_yt_comparison(_):
    fig = go.Figure()
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['odom.pos.y'], 
            mode='lines', 
            name='CARLA Y Position',
            line=dict(color='blue', width=2),
            hovertemplate='Time: %{x:.2f}s<br>Y: %{y:.2f}m<extra></extra>'
        ))
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['odom.pos.y'], 
            mode='lines', 
            name='PID Y Position',
            line=dict(color='red', width=2),
            hovertemplate='Time: %{x:.2f}s<br>Y: %{y:.2f}m<extra></extra>'
        ))
    
    fig.update_layout(
        title='Y Position vs. Time Comparison',
        xaxis_title='Time (s)',
        yaxis_title='Y Position (m)',
        hovermode='x unified',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

@app.callback(
    Output('odom-xy-comparison', 'figure'),
    [Input('odom-xy-comparison', 'id')]
)
def update_odom_xy_comparison(_):
    fig = go.Figure()
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['odom.pos.x'], 
            y=carla_df['odom.pos.y'], 
            mode='lines+markers', 
            name='CARLA Trajectory',
            line=dict(color='blue', width=2),
            marker=dict(size=4),
            hovertemplate='X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
        ))
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['odom.pos.x'], 
            y=pid_df['odom.pos.y'], 
            mode='lines+markers', 
            name='PID Trajectory',
            line=dict(color='red', width=2),
            marker=dict(size=4),
            hovertemplate='X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
        ))
    
    fig.update_layout(
        title='Trajectory Comparison (Y vs X Position)',
        xaxis_title='X Position (m)',
        yaxis_title='Y Position (m)',
        legend=dict(x=0.02, y=0.98)
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    
    return fig

@app.callback(
    Output('gnss-lonlat-t-comparison', 'figure'),
    [Input('gnss-lonlat-t-comparison', 'id')]
)
def update_gnss_lonlat_t_comparison(_):
    fig = make_subplots(rows=2, cols=1, subplot_titles=('Longitude vs Time', 'Latitude vs Time'))
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['gnss.longitude'], 
            mode='lines', 
            name='CARLA Longitude',
            line=dict(color='blue', width=2)
        ), row=1, col=1)
        
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['gnss.latitude'], 
            mode='lines', 
            name='CARLA Latitude',
            line=dict(color='blue', width=2)
        ), row=2, col=1)
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['gnss.longitude'], 
            mode='lines', 
            name='PID Longitude',
            line=dict(color='red', width=2)
        ), row=1, col=1)
        
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['gnss.latitude'], 
            mode='lines', 
            name='PID Latitude',
            line=dict(color='red', width=2)
        ), row=2, col=1)
    
    fig.update_layout(
        title='GNSS Coordinates vs Time Comparison',
        height=600,
        showlegend=True
    )
    
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="Longitude (°)", row=1, col=1)
    fig.update_yaxes(title_text="Latitude (°)", row=2, col=1)
    
    return fig

@app.callback(
    Output('gnss-lonlat-comparison', 'figure'),
    [Input('gnss-lonlat-comparison', 'id')]
)
def update_gnss_lonlat_comparison(_):
    fig = go.Figure()
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['gnss.longitude'], 
            y=carla_df['gnss.latitude'], 
            mode='markers', 
            name='CARLA GNSS',
            marker=dict(size=4, color='blue'),
            hovertemplate='Lon: %{x:.6f}°<br>Lat: %{y:.6f}°<extra></extra>'
        ))
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['gnss.longitude'], 
            y=pid_df['gnss.latitude'], 
            mode='markers', 
            name='PID GNSS',
            marker=dict(size=4, color='red'),
            hovertemplate='Lon: %{x:.6f}°<br>Lat: %{y:.6f}°<extra></extra>'
        ))
    
    fig.update_layout(
        title='GNSS Coordinates Comparison (Latitude vs Longitude)',
        xaxis_title='Longitude (°)',
        yaxis_title='Latitude (°)',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

@app.callback(
    Output('imu-accel-comparison', 'figure'),
    [Input('imu-accel-comparison', 'id')]
)
def update_imu_accel_comparison(_):
    fig = make_subplots(rows=3, cols=1, subplot_titles=('X Acceleration', 'Y Acceleration', 'Z Acceleration'))
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['imu.linear.x'], 
            mode='lines', 
            name='CARLA ax',
            line=dict(color='blue', width=2)
        ), row=1, col=1)
        
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['imu.linear.y'], 
            mode='lines', 
            name='CARLA ay',
            line=dict(color='blue', width=2)
        ), row=2, col=1)
        
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['imu.linear.z'], 
            mode='lines', 
            name='CARLA az',
            line=dict(color='blue', width=2)
        ), row=3, col=1)
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['imu.linear.x'], 
            mode='lines', 
            name='PID ax',
            line=dict(color='red', width=2)
        ), row=1, col=1)
        
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['imu.linear.y'], 
            mode='lines', 
            name='PID ay',
            line=dict(color='red', width=2)
        ), row=2, col=1)
        
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['imu.linear.z'], 
            mode='lines', 
            name='PID az',
            line=dict(color='red', width=2)
        ), row=3, col=1)
    
    fig.update_layout(
        title='IMU Linear Acceleration Comparison',
        height=600,
        showlegend=True
    )
    
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="m/s²", row=1, col=1)
    fig.update_yaxes(title_text="m/s²", row=2, col=1)
    fig.update_yaxes(title_text="m/s²", row=3, col=1)
    
    return fig

@app.callback(
    Output('speed-comparison', 'figure'),
    [Input('speed-comparison', 'id')]
)
def update_speed_comparison(_):
    fig = go.Figure()
    
    if not carla_df.empty:
        fig.add_trace(go.Scatter(
            x=carla_df['time_sec'], 
            y=carla_df['speed'], 
            mode='lines', 
            name='CARLA Speed',
            line=dict(color='blue', width=2),
            hovertemplate='Time: %{x:.2f}s<br>Speed: %{y:.2f} m/s<extra></extra>'
        ))
    
    if not pid_df.empty:
        fig.add_trace(go.Scatter(
            x=pid_df['time_sec'], 
            y=pid_df['speed'], 
            mode='lines', 
            name='PID Speed',
            line=dict(color='red', width=2),
            hovertemplate='Time: %{x:.2f}s<br>Speed: %{y:.2f} m/s<extra></extra>'
        ))
    
    fig.update_layout(
        title='Vehicle Speed Comparison',
        xaxis_title='Time (s)',
        yaxis_title='Speed (m/s)',
        hovermode='x unified',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

if __name__ == '__main__':
    print("Starting CARLA and PID Data Comparison Dashboard...")
    print(f"CARLA data points: {len(carla_df)}")
    print(f"PID data points: {len(pid_df)}")
    app.run(debug=True, host='0.0.0.0', port=8051)