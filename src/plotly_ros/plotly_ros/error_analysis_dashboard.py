#!/usr/bin/env python3

'''
Real-Time Error Analysis Dashboard for CARLA and PID Data

This dashboard calculates and displays:
1. Real-time distance error (Euclidean distance between positions)
2. Real-time velocity error (difference in speed and velocity components)
3. Root Mean Square Error (RMSE) for position and velocity
4. Average error and standard deviation
5. Error trends over time
6. Statistical error analysis

Features:
- Real-time error calculation and visualization
- Interactive plots with error metrics
- Statistical analysis panels
- Error distribution histograms
- Performance comparison metrics
'''

import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dash import Dash, dcc, html, Input, Output
import os
from scipy import stats
import math

# --- Data Loading and Preprocessing ---

def preprocess_data(filepath):
    """
    Loads and preprocesses data from a CSV file.
    
    This function:
    1. Reads CSV data and converts nanosecond timestamps to seconds
    2. Forward-fills missing values from different sensor rates
    3. Drops rows with missing essential data (position, velocity, speed)
    
    Returns: DataFrame with cleaned and preprocessed data
    """
    try:
        df = pd.read_csv(filepath)
        # Convert timestamp to seconds
        df['time_sec'] = df['timestamp_ns'] / 1e9
        # Forward-fill to handle NaNs from different sensor rates
        df.ffill(inplace=True)
        # Drop rows where essential data is missing
        df.dropna(subset=['odom.pos.x', 'odom.pos.y', 'odom.vel.x', 'odom.vel.y', 'speed'], inplace=True)
        
        return df
    except FileNotFoundError:
        print(f"Warning: File not found at {filepath}. Creating an empty DataFrame.")
        return pd.DataFrame(columns=[
            'time_sec', 'odom.pos.x', 'odom.pos.y', 'odom.vel.x', 'odom.vel.y', 'speed'
        ])

def align_datasets_by_position(carla_df, pid_df):
    """
    Aligns datasets by finding the best matching X positions and adjusting time accordingly.
    
    ERROR ALIGNMENT METHOD:
    1. Find overlapping X position ranges between CARLA and PID datasets
    2. Choose a reference X position in the middle of the overlap
    3. Find the closest X positions in both datasets to this reference
    4. Calculate time offset to align at the reference position
    5. Apply time offset to PID data and shift both datasets to start from 0
    
    This ensures fair comparison by aligning datasets at the same spatial position
    rather than just starting from time 0, which may not represent the same location.
    
    Args:
        carla_df: DataFrame with CARLA data (ground truth)
        pid_df: DataFrame with PID controller data
    
    Returns:
        carla_df, pid_df: Aligned datasets with synchronized timestamps
    """
    if carla_df.empty or pid_df.empty:
        print("Warning: One or both datasets are empty. Cannot perform alignment.")
        return carla_df, pid_df
    
    print("Performing position-based time alignment...")
    
    # Get the range of X positions that overlap between both datasets
    carla_x_min, carla_x_max = carla_df['odom.pos.x'].min(), carla_df['odom.pos.x'].max()
    pid_x_min, pid_x_max = pid_df['odom.pos.x'].min(), pid_df['odom.pos.x'].max()
    
    print(f"CARLA X range: {carla_x_min:.2f} to {carla_x_max:.2f}")
    print(f"PID X range: {pid_x_min:.2f} to {pid_x_max:.2f}")
    
    # Find overlapping X range
    overlap_min = max(carla_x_min, pid_x_min)
    overlap_max = min(carla_x_max, pid_x_max)
    
    if overlap_min >= overlap_max:
        print("Warning: No overlapping X positions found. Using simple time alignment.")
        # Fallback to simple time alignment
        carla_df['time_sec'] = carla_df['time_sec'] - carla_df['time_sec'].min()
        pid_df['time_sec'] = pid_df['time_sec'] - pid_df['time_sec'].min()
        return carla_df, pid_df
    
    print(f"Overlapping X range: {overlap_min:.2f} to {overlap_max:.2f}")
    
    # Find a reference X position in the middle of the overlap
    reference_x = (overlap_min + overlap_max) / 2
    print(f"Reference X position: {reference_x:.2f}")
    
    # Find the closest X positions in both datasets
    carla_closest_idx = (carla_df['odom.pos.x'] - reference_x).abs().idxmin()
    pid_closest_idx = (pid_df['odom.pos.x'] - reference_x).abs().idxmin()
    
    carla_ref_time = carla_df.loc[carla_closest_idx, 'time_sec']
    pid_ref_time = pid_df.loc[pid_closest_idx, 'time_sec']
    
    carla_ref_x = carla_df.loc[carla_closest_idx, 'odom.pos.x']
    pid_ref_x = pid_df.loc[pid_closest_idx, 'odom.pos.x']
    
    print(f"CARLA reference: X={carla_ref_x:.2f}, Time={carla_ref_time:.2f}")
    print(f"PID reference: X={pid_ref_x:.2f}, Time={pid_ref_time:.2f}")
    
    # Calculate time offset to align at the reference position
    time_offset = carla_ref_time - pid_ref_time
    print(f"Time offset to align: {time_offset:.2f}s")
    
    # Apply the time offset to PID data
    pid_df['time_sec'] = pid_df['time_sec'] + time_offset
    
    # Now shift both datasets to start from 0
    min_time = min(carla_df['time_sec'].min(), pid_df['time_sec'].min())
    carla_df['time_sec'] = carla_df['time_sec'] - min_time
    pid_df['time_sec'] = pid_df['time_sec'] - min_time
    
    print(f"Final alignment: CARLA starts at {carla_df['time_sec'].min():.2f}s, PID starts at {pid_df['time_sec'].min():.2f}s")
    
    return carla_df, pid_df

def calculate_errors(carla_df, pid_df):
    """
    Calculates various error metrics between CARLA and PID datasets.
    
    ERROR CALCULATION METHOD:
    CARLA data is treated as "ground truth" and PID data is compared against it.
    
    POSITION ERRORS:
    - pos_error_x = carla_x - pid_x (positive = CARLA ahead in X direction)
    - pos_error_y = carla_y - pid_y (positive = CARLA ahead in Y direction)
    - distance_error = √((carla_x - pid_x)² + (carla_y - pid_y)²) (always positive)
    
    VELOCITY ERRORS:
    - vel_error_x = carla_vx - pid_vx (positive = CARLA faster in X direction)
    - vel_error_y = carla_vy - pid_vy (positive = CARLA faster in Y direction)
    - speed_error = carla_speed - pid_speed (positive = CARLA faster)
    - velocity_magnitude_error = √((carla_vx - pid_vx)² + (carla_vy - pid_vy)²) (always positive)
    
    INTERPOLATION:
    PID data is interpolated to match CARLA timestamps for fair comparison.
    
    Args:
        carla_df: DataFrame with CARLA data (ground truth)
        pid_df: DataFrame with PID controller data
    
    Returns:
        DataFrame with all error metrics calculated
    """
    if carla_df.empty or pid_df.empty:
        return pd.DataFrame()
    
    # Interpolate PID data to match CARLA timestamps for fair comparison
    carla_times = carla_df['time_sec'].values
    
    # Create interpolation functions for PID data
    pid_interp_x = np.interp(carla_times, pid_df['time_sec'], pid_df['odom.pos.x'])
    pid_interp_y = np.interp(carla_times, pid_df['time_sec'], pid_df['odom.pos.y'])
    pid_interp_vx = np.interp(carla_times, pid_df['time_sec'], pid_df['odom.vel.x'])
    pid_interp_vy = np.interp(carla_times, pid_df['time_sec'], pid_df['odom.vel.y'])
    pid_interp_speed = np.interp(carla_times, pid_df['time_sec'], pid_df['speed'])
    
    # Calculate errors
    errors_df = pd.DataFrame({
        'time_sec': carla_times,
        'carla_x': carla_df['odom.pos.x'].values,
        'carla_y': carla_df['odom.pos.y'].values,
        'carla_vx': carla_df['odom.vel.x'].values,
        'carla_vy': carla_df['odom.vel.y'].values,
        'carla_speed': carla_df['speed'].values,
        'pid_x': pid_interp_x,
        'pid_y': pid_interp_y,
        'pid_vx': pid_interp_vx,
        'pid_vy': pid_interp_vy,
        'pid_speed': pid_interp_speed,
    })
    
    # Position errors - CARLA is ground truth, PID is compared against it
    errors_df['pos_error_x'] = errors_df['carla_x'] - errors_df['pid_x']  # Positive = CARLA ahead in X
    errors_df['pos_error_y'] = errors_df['carla_y'] - errors_df['pid_y']  # Positive = CARLA ahead in Y
    errors_df['distance_error'] = np.sqrt(errors_df['pos_error_x']**2 + errors_df['pos_error_y']**2)  # Always positive
    
    # Velocity errors - CARLA is ground truth, PID is compared against it
    errors_df['vel_error_x'] = errors_df['carla_vx'] - errors_df['pid_vx']  # Positive = CARLA faster in X
    errors_df['vel_error_y'] = errors_df['carla_vy'] - errors_df['pid_vy']  # Positive = CARLA faster in Y
    errors_df['speed_error'] = errors_df['carla_speed'] - errors_df['pid_speed']  # Positive = CARLA faster
    errors_df['velocity_magnitude_error'] = np.sqrt(errors_df['vel_error_x']**2 + errors_df['vel_error_y']**2)  # Always positive
    
    # Calculate velocity magnitude for both datasets
    errors_df['carla_vel_mag'] = np.sqrt(errors_df['carla_vx']**2 + errors_df['carla_vy']**2)
    errors_df['pid_vel_mag'] = np.sqrt(errors_df['pid_vx']**2 + errors_df['pid_vy']**2)
    errors_df['vel_mag_error'] = errors_df['carla_vel_mag'] - errors_df['pid_vel_mag']  # Positive = CARLA faster
    
    return errors_df

def calculate_statistics(errors_df):
    """
    Calculates comprehensive error statistics from the error DataFrame.
    
    STATISTICAL METRICS CALCULATED:
    
    POSITION ERROR STATISTICS:
    - pos_rmse_x: Root Mean Square Error of X position errors
    - pos_rmse_y: Root Mean Square Error of Y position errors  
    - distance_rmse: Root Mean Square Error of distance errors
    - pos_mean_error_x: Mean X position error
    - pos_mean_error_y: Mean Y position error
    - distance_mean_error: Mean distance error
    - pos_std_error_x: Standard deviation of X position errors
    - pos_std_error_y: Standard deviation of Y position errors
    - distance_std_error: Standard deviation of distance errors
    
    VELOCITY ERROR STATISTICS:
    - vel_rmse_x: Root Mean Square Error of X velocity errors
    - vel_rmse_y: Root Mean Square Error of Y velocity errors
    - speed_rmse: Root Mean Square Error of speed errors
    - vel_mag_rmse: Root Mean Square Error of velocity magnitude errors
    - vel_mean_error_x: Mean X velocity error
    - vel_mean_error_y: Mean Y velocity error
    - speed_mean_error: Mean speed error
    - vel_mag_mean_error: Mean velocity magnitude error
    - vel_std_error_x: Standard deviation of X velocity errors
    - vel_std_error_y: Standard deviation of Y velocity errors
    - speed_std_error: Standard deviation of speed errors
    - vel_mag_std_error: Standard deviation of velocity magnitude errors
    
    ADDITIONAL STATISTICS:
    - max_distance_error: Maximum distance error value
    - min_distance_error: Minimum distance error value
    - max_speed_error: Maximum speed error value
    - min_speed_error: Minimum speed error value
    - pos_correlation: Correlation coefficient between CARLA and PID X positions
    - speed_correlation: Correlation coefficient between CARLA and PID speeds
    
    RMSE FORMULA: √(mean(error²)) - measures the magnitude of errors
    CORRELATION: Measures how well CARLA and PID data follow similar patterns (1 = perfect correlation)
    
    Args:
        errors_df: DataFrame with calculated error metrics
    
    Returns:
        Dictionary containing all statistical metrics
    """
    if errors_df.empty:
        return {}
    
    stats_dict = {}
    
    # Position error statistics
    stats_dict['pos_rmse_x'] = np.sqrt(np.mean(errors_df['pos_error_x']**2))  # RMSE of X position errors
    stats_dict['pos_rmse_y'] = np.sqrt(np.mean(errors_df['pos_error_y']**2))  # RMSE of Y position errors
    stats_dict['distance_rmse'] = np.sqrt(np.mean(errors_df['distance_error']**2))  # RMSE of distance errors
    
    stats_dict['pos_mean_error_x'] = np.mean(errors_df['pos_error_x'])  # Mean X position error
    stats_dict['pos_mean_error_y'] = np.mean(errors_df['pos_error_y'])  # Mean Y position error
    stats_dict['distance_mean_error'] = np.mean(errors_df['distance_error'])  # Mean distance error
    
    stats_dict['pos_std_error_x'] = np.std(errors_df['pos_error_x'])  # Std dev of X position errors
    stats_dict['pos_std_error_y'] = np.std(errors_df['pos_error_y'])  # Std dev of Y position errors
    stats_dict['distance_std_error'] = np.std(errors_df['distance_error'])  # Std dev of distance errors
    
    # Velocity error statistics
    stats_dict['vel_rmse_x'] = np.sqrt(np.mean(errors_df['vel_error_x']**2))  # RMSE of X velocity errors
    stats_dict['vel_rmse_y'] = np.sqrt(np.mean(errors_df['vel_error_y']**2))  # RMSE of Y velocity errors
    stats_dict['speed_rmse'] = np.sqrt(np.mean(errors_df['speed_error']**2))  # RMSE of speed errors
    stats_dict['vel_mag_rmse'] = np.sqrt(np.mean(errors_df['velocity_magnitude_error']**2))  # RMSE of velocity magnitude errors
    
    stats_dict['vel_mean_error_x'] = np.mean(errors_df['vel_error_x'])  # Mean X velocity error
    stats_dict['vel_mean_error_y'] = np.mean(errors_df['vel_error_y'])  # Mean Y velocity error
    stats_dict['speed_mean_error'] = np.mean(errors_df['speed_error'])  # Mean speed error
    stats_dict['vel_mag_mean_error'] = np.mean(errors_df['velocity_magnitude_error'])  # Mean velocity magnitude error
    
    stats_dict['vel_std_error_x'] = np.std(errors_df['vel_error_x'])  # Std dev of X velocity errors
    stats_dict['vel_std_error_y'] = np.std(errors_df['vel_error_y'])  # Std dev of Y velocity errors
    stats_dict['speed_std_error'] = np.std(errors_df['speed_error'])  # Std dev of speed errors
    stats_dict['vel_mag_std_error'] = np.std(errors_df['velocity_magnitude_error'])  # Std dev of velocity magnitude errors
    
    # Additional statistics
    stats_dict['max_distance_error'] = np.max(errors_df['distance_error'])  # Maximum distance error
    stats_dict['min_distance_error'] = np.min(errors_df['distance_error'])  # Minimum distance error
    stats_dict['max_speed_error'] = np.max(errors_df['speed_error'])  # Maximum speed error
    stats_dict['min_speed_error'] = np.min(errors_df['speed_error'])  # Minimum speed error
    
    # Correlation coefficients - measures how well CARLA and PID data correlate
    stats_dict['pos_correlation'] = np.corrcoef(errors_df['carla_x'], errors_df['pid_x'])[0, 1]  # Position correlation
    stats_dict['speed_correlation'] = np.corrcoef(errors_df['carla_speed'], errors_df['pid_speed'])[0, 1]  # Speed correlation
    
    return stats_dict

# Load and process data
csv_folder = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV'
carla_df = preprocess_data(os.path.join(csv_folder, 'carla_merged_data.csv'))
pid_df = preprocess_data(os.path.join(csv_folder, 'pid_merged.csv'))

# Align datasets
carla_df, pid_df = align_datasets_by_position(carla_df, pid_df)

# Calculate errors
errors_df = calculate_errors(carla_df, pid_df)
error_stats = calculate_statistics(errors_df)

print(f"CARLA data loaded: {len(carla_df)} rows")
print(f"PID data loaded: {len(pid_df)} rows")
print(f"Error analysis performed on {len(errors_df)} data points")

# --- Dash App Initialization ---

app = Dash(__name__)

app.layout = html.Div([
    html.H1("Real-Time Error Analysis Dashboard", style={'textAlign': 'center'}),
    
    # Dataset info
    html.Div([
        html.H3("Dataset Information", style={'textAlign': 'center'}),
        html.P(f"CARLA Dataset: {len(carla_df)} points", style={'textAlign': 'center'}),
        html.P(f"PID Dataset: {len(pid_df)} points", style={'textAlign': 'center'}),
        html.P(f"Error Analysis: {len(errors_df)} aligned points", style={'textAlign': 'center'}),
    ], style={'marginBottom': '20px'}),
    
    # Error Statistics Summary
    html.Div([
        html.H3("Error Statistics Summary", style={'textAlign': 'center'}),
        html.Div([
            html.Div([
                html.H4("Position Errors"),
                html.P(f"Distance RMSE: {error_stats.get('distance_rmse', 0):.4f} m"),
                html.P(f"Distance Mean Error: {error_stats.get('distance_mean_error', 0):.4f} m"),
                html.P(f"Distance Std Dev: {error_stats.get('distance_std_error', 0):.4f} m"),
                html.P(f"X Position RMSE: {error_stats.get('pos_rmse_x', 0):.4f} m"),
                html.P(f"Y Position RMSE: {error_stats.get('pos_rmse_y', 0):.4f} m"),
            ], style={'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top'}),
            
            html.Div([
                html.H4("Velocity Errors"),
                html.P(f"Speed RMSE: {error_stats.get('speed_rmse', 0):.4f} m/s"),
                html.P(f"Speed Mean Error: {error_stats.get('speed_mean_error', 0):.4f} m/s"),
                html.P(f"Speed Std Dev: {error_stats.get('speed_std_error', 0):.4f} m/s"),
                html.P(f"Velocity Magnitude RMSE: {error_stats.get('vel_mag_rmse', 0):.4f} m/s"),
                html.P(f"X Velocity RMSE: {error_stats.get('vel_rmse_x', 0):.4f} m/s"),
            ], style={'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top'}),
            
            html.Div([
                html.H4("Correlation Analysis"),
                html.P(f"Position Correlation: {error_stats.get('pos_correlation', 0):.4f}"),
                html.P(f"Speed Correlation: {error_stats.get('speed_correlation', 0):.4f}"),
                html.P(f"Max Distance Error: {error_stats.get('max_distance_error', 0):.4f} m"),
                html.P(f"Max Speed Error: {error_stats.get('max_speed_error', 0):.4f} m/s"),
            ], style={'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top'}),
        ], style={'textAlign': 'center'}),
    ], style={'marginBottom': '30px', 'padding': '20px', 'backgroundColor': '#f8f9fa', 'borderRadius': '10px'}),
    
    # Real-time Error Plots
    html.H2("Real-Time Error Analysis", style={'textAlign': 'center', 'marginTop': '40px'}),
    
    # Distance Error vs Time
    dcc.Graph(id='distance-error-plot'),
    
    # Velocity Error vs Time
    dcc.Graph(id='velocity-error-plot'),
    
    # Position Error Components
    dcc.Graph(id='position-error-components'),
    
    # Velocity Error Components
    dcc.Graph(id='velocity-error-components'),
    
    # Error Distribution Histograms
    html.H2("Error Distribution Analysis", style={'textAlign': 'center', 'marginTop': '40px'}),
    dcc.Graph(id='error-distribution-histograms'),
    
    # Cumulative Error Analysis
    html.H2("Cumulative Error Analysis", style={'textAlign': 'center', 'marginTop': '40px'}),
    dcc.Graph(id='cumulative-error-analysis'),
    
    # Performance Metrics Over Time
    html.H2("Performance Metrics Over Time", style={'textAlign': 'center', 'marginTop': '40px'}),
    dcc.Graph(id='performance-metrics'),
    
], style={'width': '95%', 'margin': 'auto'})

# --- Callbacks ---

@app.callback(
    Output('distance-error-plot', 'figure'),
    [Input('distance-error-plot', 'id')]
)
def update_distance_error_plot(_):
    """
    Creates a plot showing distance error over time.
    
    PLOT FEATURES:
    - Distance error line (red): Shows how far apart CARLA and PID positions are
    - Mean error line (orange dashed): Average distance error across all time points
    - RMSE line (purple dotted): Root Mean Square Error of distance errors
    
    INTERPRETATION:
    - Lower values = better performance (PID follows CARLA trajectory closely)
    - Mean line shows average performance
    - RMSE line shows error magnitude (higher = more variable errors)
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['distance_error'],
        mode='lines',
        name='Distance Error',
        line=dict(color='red', width=2),
        hovertemplate='Time: %{x:.2f}s<br>Distance Error: %{y:.4f}m<extra></extra>'
    ))
    
    # Add mean line
    mean_error = np.mean(errors_df['distance_error'])
    fig.add_hline(y=mean_error, line_dash="dash", line_color="orange", 
                  annotation_text=f"Mean: {mean_error:.4f}m")
    
    # Add RMSE line
    rmse_error = np.sqrt(np.mean(errors_df['distance_error']**2))
    fig.add_hline(y=rmse_error, line_dash="dot", line_color="purple", 
                  annotation_text=f"RMSE: {rmse_error:.4f}m")
    
    fig.update_layout(
        title='Real-Time Distance Error vs Time',
        xaxis_title='Time (s)',
        yaxis_title='Distance Error (m)',
        hovermode='x unified',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

@app.callback(
    Output('velocity-error-plot', 'figure'),
    [Input('velocity-error-plot', 'id')]
)
def update_velocity_error_plot(_):
    """
    Creates a plot showing velocity errors over time.
    
    PLOT FEATURES:
    - Speed error line (blue): Difference between CARLA and PID speed values
    - Velocity magnitude error line (green): Magnitude of velocity vector differences
    - Mean lines: Average errors for both metrics
    
    INTERPRETATION:
    - Speed error: Positive = CARLA faster, Negative = PID faster
    - Velocity magnitude error: Always positive, shows overall velocity difference
    - Lower values = better performance (PID matches CARLA velocity)
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['speed_error'],
        mode='lines',
        name='Speed Error',
        line=dict(color='blue', width=2),
        hovertemplate='Time: %{x:.2f}s<br>Speed Error: %{y:.4f}m/s<extra></extra>'
    ))
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['velocity_magnitude_error'],
        mode='lines',
        name='Velocity Magnitude Error',
        line=dict(color='green', width=2),
        hovertemplate='Time: %{x:.2f}s<br>Velocity Magnitude Error: %{y:.4f}m/s<extra></extra>'
    ))
    
    # Add mean lines
    mean_speed_error = np.mean(errors_df['speed_error'])
    mean_vel_mag_error = np.mean(errors_df['velocity_magnitude_error'])
    
    fig.add_hline(y=mean_speed_error, line_dash="dash", line_color="orange", 
                  annotation_text=f"Speed Mean: {mean_speed_error:.4f}m/s")
    fig.add_hline(y=mean_vel_mag_error, line_dash="dash", line_color="red", 
                  annotation_text=f"Vel Mag Mean: {mean_vel_mag_error:.4f}m/s")
    
    fig.update_layout(
        title='Real-Time Velocity Error vs Time',
        xaxis_title='Time (s)',
        yaxis_title='Velocity Error (m/s)',
        hovermode='x unified',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

@app.callback(
    Output('position-error-components', 'figure'),
    [Input('position-error-components', 'id')]
)
def update_position_error_components(_):
    """
    Creates subplots showing X and Y position errors separately.
    
    PLOT FEATURES:
    - X Position Error (top): Difference in X coordinates (CARLA - PID)
    - Y Position Error (bottom): Difference in Y coordinates (CARLA - PID)
    - Mean lines: Average errors for each component
    
    INTERPRETATION:
    - Positive X error = CARLA ahead in X direction
    - Positive Y error = CARLA ahead in Y direction
    - Negative errors = PID ahead in that direction
    - Helps identify which direction has larger errors
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = make_subplots(rows=2, cols=1, subplot_titles=('X Position Error', 'Y Position Error'))
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['pos_error_x'],
        mode='lines',
        name='X Position Error',
        line=dict(color='red', width=2)
    ), row=1, col=1)
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['pos_error_y'],
        mode='lines',
        name='Y Position Error',
        line=dict(color='blue', width=2)
    ), row=2, col=1)
    
    # Add mean lines
    mean_x_error = np.mean(errors_df['pos_error_x'])
    mean_y_error = np.mean(errors_df['pos_error_y'])
    
    fig.add_hline(y=mean_x_error, line_dash="dash", line_color="orange", row=1, col=1,
                  annotation_text=f"Mean: {mean_x_error:.4f}m")
    fig.add_hline(y=mean_y_error, line_dash="dash", line_color="orange", row=2, col=1,
                  annotation_text=f"Mean: {mean_y_error:.4f}m")
    
    fig.update_layout(
        title='Position Error Components vs Time',
        height=600,
        showlegend=True
    )
    
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="X Position Error (m)", row=1, col=1)
    fig.update_yaxes(title_text="Y Position Error (m)", row=2, col=1)
    
    return fig

@app.callback(
    Output('velocity-error-components', 'figure'),
    [Input('velocity-error-components', 'id')]
)
def update_velocity_error_components(_):
    """
    Creates subplots showing X and Y velocity errors separately.
    
    PLOT FEATURES:
    - X Velocity Error (top): Difference in X velocity components (CARLA - PID)
    - Y Velocity Error (bottom): Difference in Y velocity components (CARLA - PID)
    - Mean lines: Average errors for each component
    
    INTERPRETATION:
    - Positive X velocity error = CARLA faster in X direction
    - Positive Y velocity error = CARLA faster in Y direction
    - Negative errors = PID faster in that direction
    - Helps identify velocity control issues in specific directions
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = make_subplots(rows=2, cols=1, subplot_titles=('X Velocity Error', 'Y Velocity Error'))
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['vel_error_x'],
        mode='lines',
        name='X Velocity Error',
        line=dict(color='red', width=2)
    ), row=1, col=1)
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=errors_df['vel_error_y'],
        mode='lines',
        name='Y Velocity Error',
        line=dict(color='blue', width=2)
    ), row=2, col=1)
    
    # Add mean lines
    mean_vx_error = np.mean(errors_df['vel_error_x'])
    mean_vy_error = np.mean(errors_df['vel_error_y'])
    
    fig.add_hline(y=mean_vx_error, line_dash="dash", line_color="orange", row=1, col=1,
                  annotation_text=f"Mean: {mean_vx_error:.4f}m/s")
    fig.add_hline(y=mean_vy_error, line_dash="dash", line_color="orange", row=2, col=1,
                  annotation_text=f"Mean: {mean_vy_error:.4f}m/s")
    
    fig.update_layout(
        title='Velocity Error Components vs Time',
        height=600,
        showlegend=True
    )
    
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="X Velocity Error (m/s)", row=1, col=1)
    fig.update_yaxes(title_text="Y Velocity Error (m/s)", row=2, col=1)
    
    return fig

@app.callback(
    Output('error-distribution-histograms', 'figure'),
    [Input('error-distribution-histograms', 'id')]
)
def update_error_distribution_histograms(_):
    """
    Creates histograms showing the distribution of different error types.
    
    PLOT FEATURES:
    - Distance Error Distribution: Shows how distance errors are distributed
    - Speed Error Distribution: Shows how speed errors are distributed
    - X Position Error Distribution: Shows X position error distribution
    - Y Position Error Distribution: Shows Y position error distribution
    
    INTERPRETATION:
    - Centered around 0 = balanced errors (no systematic bias)
    - Skewed distributions = systematic bias in one direction
    - Narrow distributions = consistent performance
    - Wide distributions = variable performance
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = make_subplots(rows=2, cols=2, subplot_titles=(
        'Distance Error Distribution', 'Speed Error Distribution',
        'X Position Error Distribution', 'Y Position Error Distribution'
    ))
    
    # Distance error histogram
    fig.add_trace(go.Histogram(
        x=errors_df['distance_error'],
        nbinsx=30,
        name='Distance Error',
        marker_color='red',
        opacity=0.7
    ), row=1, col=1)
    
    # Speed error histogram
    fig.add_trace(go.Histogram(
        x=errors_df['speed_error'],
        nbinsx=30,
        name='Speed Error',
        marker_color='blue',
        opacity=0.7
    ), row=1, col=2)
    
    # X position error histogram
    fig.add_trace(go.Histogram(
        x=errors_df['pos_error_x'],
        nbinsx=30,
        name='X Position Error',
        marker_color='green',
        opacity=0.7
    ), row=2, col=1)
    
    # Y position error histogram
    fig.add_trace(go.Histogram(
        x=errors_df['pos_error_y'],
        nbinsx=30,
        name='Y Position Error',
        marker_color='purple',
        opacity=0.7
    ), row=2, col=2)
    
    fig.update_layout(
        title='Error Distribution Histograms',
        height=600,
        showlegend=False
    )
    
    fig.update_xaxes(title_text="Distance Error (m)", row=1, col=1)
    fig.update_xaxes(title_text="Speed Error (m/s)", row=1, col=2)
    fig.update_xaxes(title_text="X Position Error (m)", row=2, col=1)
    fig.update_xaxes(title_text="Y Position Error (m)", row=2, col=2)
    
    return fig

@app.callback(
    Output('cumulative-error-analysis', 'figure'),
    [Input('cumulative-error-analysis', 'id')]
)
def update_cumulative_error_analysis(_):
    """
    Creates a plot showing cumulative errors over time.
    
    PLOT FEATURES:
    - Cumulative Distance Error (left y-axis): Sum of absolute distance errors over time
    - Cumulative Speed Error (right y-axis): Sum of absolute speed errors over time
    
    INTERPRETATION:
    - Steeper slopes = higher error rates
    - Flatter slopes = better performance
    - Helps identify periods of high vs low error accumulation
    - Useful for overall performance assessment
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = go.Figure()
    
    # Calculate cumulative errors
    cumulative_distance_error = np.cumsum(np.abs(errors_df['distance_error']))
    cumulative_speed_error = np.cumsum(np.abs(errors_df['speed_error']))
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=cumulative_distance_error,
        mode='lines',
        name='Cumulative Distance Error',
        line=dict(color='red', width=2)
    ))
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=cumulative_speed_error,
        mode='lines',
        name='Cumulative Speed Error',
        line=dict(color='blue', width=2),
        yaxis='y2'
    ))
    
    fig.update_layout(
        title='Cumulative Error Analysis',
        xaxis_title='Time (s)',
        yaxis=dict(title='Cumulative Distance Error (m)', side='left'),
        yaxis2=dict(title='Cumulative Speed Error (m/s)', side='right', overlaying='y'),
        hovermode='x unified',
        legend=dict(x=0.02, y=0.98)
    )
    
    return fig

@app.callback(
    Output('performance-metrics', 'figure'),
    [Input('performance-metrics', 'id')]
)
def update_performance_metrics(_):
    """
    Creates subplots showing rolling performance metrics over time.
    
    PLOT FEATURES:
    - Rolling RMSE (top): RMSE calculated over a sliding window
    - Rolling Mean Error (bottom): Mean error calculated over a sliding window
    - Window size: Automatically adjusted based on data size (min 2, max 20)
    
    INTERPRETATION:
    - Rolling metrics show performance trends over time
    - Helps identify periods of good vs poor performance
    - Smoothing effect reduces noise in the data
    - Useful for detecting performance degradation or improvement
    """
    if errors_df.empty:
        return go.Figure().add_annotation(text="No data available", xref="paper", yref="paper", x=0.5, y=0.5, showarrow=False)
    
    fig = make_subplots(rows=2, cols=1, subplot_titles=('Rolling RMSE (20-point window)', 'Rolling Mean Error (20-point window)'))
    
    # Calculate rolling statistics
    window_size = min(20, len(errors_df) // 4)
    if window_size < 2:
        window_size = 2
    
    rolling_distance_rmse = errors_df['distance_error'].rolling(window=window_size).apply(lambda x: np.sqrt(np.mean(x**2)))
    rolling_speed_rmse = errors_df['speed_error'].rolling(window=window_size).apply(lambda x: np.sqrt(np.mean(x**2)))
    
    rolling_distance_mean = errors_df['distance_error'].rolling(window=window_size).mean()
    rolling_speed_mean = errors_df['speed_error'].rolling(window=window_size).mean()
    
    # RMSE plot
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=rolling_distance_rmse,
        mode='lines',
        name='Distance RMSE',
        line=dict(color='red', width=2)
    ), row=1, col=1)
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=rolling_speed_rmse,
        mode='lines',
        name='Speed RMSE',
        line=dict(color='blue', width=2)
    ), row=1, col=1)
    
    # Mean error plot
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=rolling_distance_mean,
        mode='lines',
        name='Distance Mean Error',
        line=dict(color='red', width=2),
        showlegend=False
    ), row=2, col=1)
    
    fig.add_trace(go.Scatter(
        x=errors_df['time_sec'],
        y=rolling_speed_mean,
        mode='lines',
        name='Speed Mean Error',
        line=dict(color='blue', width=2),
        showlegend=False
    ), row=2, col=1)
    
    fig.update_layout(
        title=f'Performance Metrics Over Time (Window Size: {window_size})',
        height=600,
        showlegend=True
    )
    
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="RMSE", row=1, col=1)
    fig.update_yaxes(title_text="Mean Error", row=2, col=1)
    
    return fig

if __name__ == '__main__':
    print("Starting Real-Time Error Analysis Dashboard...")
    print(f"Error analysis performed on {len(errors_df)} data points")
    print("Available error statistics:")
    for key, value in error_stats.items():
        print(f"  {key}: {value:.6f}")
    
    app.run(debug=True, host='0.0.0.0', port=8052) 