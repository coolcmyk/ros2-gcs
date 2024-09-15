import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const ROSComponent = () => {
  const [connected, setConnected] = useState(false);
  const [number, setNumber] = useState(0);

  useEffect(() => {
    // Inisialisasi koneksi ROS
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090', // ROS WebSocket URL
    });

    // Menangani status koneksi
    ros.on('connection', () => {
      console.log('Connected to WebSocket server.');
      setConnected(true);
    });

    ros.on('error', (error) => {
      console.log('Error connecting to WebSocket server:', error);
      setConnected(false);
    });

    ros.on('close', () => {
      console.log('Connection to WebSocket server closed.');
      setConnected(false);
    });

    // Subscribe ke topic "/angka"
    const angkaTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/angka',
      messageType: 'std_msgs/Int32'
    });

    angkaTopic.subscribe(function(message) {
      console.log('Angka diterima: ' + message.data);
      setNumber((prev) => prev + message.data);
    });

    // Cleanup ketika komponen unmount
    return () => {
      angkaTopic.unsubscribe();
      ros.close();
    };
  }, []); // Dependensi kosong, berarti efek ini hanya dijalankan sekali saat mount

  return (
    <div>
      <div
        id="is-pressed"
        className={`text-xs px-2 ${connected ? 'bg-success' : 'bg-error'}`}
      >
        {connected ? 'Connected to ROS :)' : 'Not connected to ROS... Please refresh!'}
      </div>
      <div>
        <p>Ini adalah angka: {number}</p>
      </div>
    </div>
  );
};

export default ROSComponent;
