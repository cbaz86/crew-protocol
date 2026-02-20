import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { MapContainer, TileLayer, Marker, Popup, Circle } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

// Fix default marker icons in react-leaflet
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png'),
  iconUrl: require('leaflet/dist/images/marker-icon.png'),
  shadowUrl: require('leaflet/dist/images/marker-shadow.png'),
});

function Dashboard() {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [robots, setRobots] = useState([]);
  const [emergency, setEmergency] = useState(null);

  // Connect to ROS
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS!');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.log('Error connecting to ROS:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS closed.');
      setConnected(false);
    });

    setRos(rosInstance);

    return () => {
      if (rosInstance) {
        rosInstance.close();
      }
    };
  }, []);

  // Subscribe to robot responses
  useEffect(() => {
    if (!ros || !connected) return;

    const robotListener = new ROSLIB.Topic({
      ros: ros,
      name: '/crew/robot_responses',
      messageType: 'crew_interfaces/msg/RobotResponse'
    });

    robotListener.subscribe((message) => {
      console.log('Robot response:', message);
      
      setRobots((prevRobots) => {
        const existingIndex = prevRobots.findIndex(r => r.robot_id === message.robot_id);
        
        if (existingIndex >= 0) {
          const updated = [...prevRobots];
          updated[existingIndex] = message;
          return updated;
        } else {
          return [...prevRobots, message];
        }
      });
    });

    return () => {
      robotListener.unsubscribe();
    };
  }, [ros, connected]);

  // Subscribe to emergency broadcasts
  useEffect(() => {
    if (!ros || !connected) return;

    const emergencyListener = new ROSLIB.Topic({
      ros: ros,
      name: '/crew/emergency_broadcast',
      messageType: 'crew_interfaces/msg/EmergencyRequest'
    });

    emergencyListener.subscribe((message) => {
      console.log('Emergency broadcast:', message);
      setEmergency(message);
    });

    return () => {
      emergencyListener.unsubscribe();
    };
  }, [ros, connected]);

  // Default map center (San Francisco)
  const mapCenter = emergency 
    ? [emergency.latitude, emergency.longitude]
    : [37.7749, -122.4194];

  return (
    <div style={{ 
      display: 'flex', 
      flexDirection: 'column', 
      height: '100vh',
      fontFamily: 'Arial, sans-serif'
    }}>
      {/* Header */}
      <div style={{
        background: '#1a1a2e',
        color: 'white',
        padding: '20px',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center'
      }}>
        <h1 style={{ margin: 0 }}>🚒 CREW Dashboard</h1>
        <div style={{
          background: connected ? '#4caf50' : '#f44336',
          padding: '8px 16px',
          borderRadius: '20px',
          fontSize: '14px'
        }}>
          {connected ? '✓ Connected to ROS' : '✗ Disconnected'}
        </div>
      </div>

      {/* Main Content */}
      <div style={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
        
        {/* Map */}
        <div style={{ flex: 2, position: 'relative' }}>
          <MapContainer 
            center={mapCenter} 
            zoom={13} 
            style={{ height: '100%', width: '100%' }}
          >
            <TileLayer
              attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            
            {/* Emergency Location */}
            {emergency && (
              <>
                <Circle
                  center={[emergency.latitude, emergency.longitude]}
                  radius={emergency.radius_km * 1000}
                  pathOptions={{ color: 'red', fillColor: 'red', fillOpacity: 0.1 }}
                />
                <Marker position={[emergency.latitude, emergency.longitude]}>
                  <Popup>
                    <strong>🔥 {emergency.emergency_type}</strong><br/>
                    Needs: {emergency.capabilities_needed.join(', ')}
                  </Popup>
                </Marker>
              </>
            )}

            {/* Robot Markers */}
            {robots.map((robot) => (
              <Marker 
                key={robot.robot_id}
                position={[robot.latitude, robot.longitude]}
              >
                <Popup>
                  <strong>🤖 {robot.robot_id}</strong><br/>
                  Type: {robot.robot_type}<br/>
                  Status: {robot.status}<br/>
                  Battery: {robot.battery_percent}%<br/>
                  Capabilities: {robot.capabilities.join(', ')}
                </Popup>
              </Marker>
            ))}
          </MapContainer>
        </div>

        {/* Sidebar */}
        <div style={{
          flex: 1,
          background: '#f5f5f5',
          padding: '20px',
          overflowY: 'auto'
        }}>
          
          {/* Emergency Info */}
          {emergency && (
            <div style={{
              background: '#fff3cd',
              border: '2px solid #ffc107',
              borderRadius: '8px',
              padding: '16px',
              marginBottom: '20px'
            }}>
              <h3 style={{ margin: '0 0 10px 0', color: '#856404' }}>
                🚨 Active Emergency
              </h3>
              <p><strong>Type:</strong> {emergency.emergency_type}</p>
              <p><strong>Location:</strong> {emergency.latitude.toFixed(4)}, {emergency.longitude.toFixed(4)}</p>
              <p><strong>Radius:</strong> {emergency.radius_km} km</p>
              <p><strong>Needs:</strong></p>
              <ul style={{ margin: '5px 0' }}>
                {emergency.capabilities_needed.map((cap, i) => (
                  <li key={i}>{cap}</li>
                ))}
              </ul>
            </div>
          )}

          {/* Robot List */}
          <h3>Available CREW ({robots.length})</h3>
          {robots.length === 0 ? (
            <p style={{ color: '#666' }}>Waiting for robots to respond...</p>
          ) : (
            robots.map((robot) => (
              <div
                key={robot.robot_id}
                style={{
                  background: 'white',
                  border: '1px solid #ddd',
                  borderRadius: '8px',
                  padding: '16px',
                  marginBottom: '12px'
                }}
              >
                <h4 style={{ 
                  margin: '0 0 10px 0',
                  color: robot.status === 'AVAILABLE' ? '#4caf50' : '#666'
                }}>
                  🤖 {robot.robot_id}
                </h4>
                <div style={{ fontSize: '14px', lineHeight: '1.6' }}>
                  <p><strong>Type:</strong> {robot.robot_type}</p>
                  <p><strong>Status:</strong> 
                    <span style={{
                      background: robot.status === 'AVAILABLE' ? '#4caf50' : '#999',
                      color: 'white',
                      padding: '2px 8px',
                      borderRadius: '4px',
                      marginLeft: '8px',
                      fontSize: '12px'
                    }}>
                      {robot.status}
                    </span>
                  </p>
                  <p><strong>Battery:</strong> {robot.battery_percent.toFixed(0)}%</p>
                  <p><strong>ETA:</strong> {robot.eta_minutes} min</p>
                  <p><strong>Capabilities:</strong></p>
                  <div style={{ display: 'flex', flexWrap: 'wrap', gap: '4px' }}>
                    {robot.capabilities.map((cap, i) => (
                      <span
                        key={i}
                        style={{
                          background: '#e3f2fd',
                          color: '#1976d2',
                          padding: '4px 8px',
                          borderRadius: '4px',
                          fontSize: '12px'
                        }}
                      >
                        {cap}
                      </span>
                    ))}
                  </div>
                </div>
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
}

export default Dashboard;
