import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const ROSComponent = () => {
  const [connected, setConnected] = useState(false);
  const [number, setNumber] = useState(0);
  const [imageData, setImageData] = useState(null);


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

    const cameraTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/camera', // Topic kamera
      messageType: 'sensor_msgs/Image', // Tipe pesan gambar
    });

    cameraTopic.subscribe((message) => {
      console.log('Gambar diterima dari topic /camera.');

      // Mengonversi data gambar ke format base64 (sebagai contoh)
      const imageWidth = message.width;
      const imageHeight = message.height;
      const imageData = message.data; // Ini adalah data raw gambar (dalam bentuk uint8array)

      // Kita asumsikan data diterima dalam format RGB 8-bit (3 channel)
      const canvas = document.createElement('canvas');
      canvas.width = imageWidth;
      canvas.height = imageHeight;
      const context = canvas.getContext('2d');
      const imgData = context.createImageData(imageWidth, imageHeight);

      for (let i = 0; i < imgData.data.length; i += 4) {
        imgData.data[i] = imageData[i];     // R
        imgData.data[i + 1] = imageData[i + 1]; // G
        imgData.data[i + 2] = imageData[i + 2]; // B
        imgData.data[i + 3] = 255;          // Alpha channel
      }

      context.putImageData(imgData, 0, 0);

      // Mengonversi canvas ke URL gambar base64
      const imageUrl = canvas.toDataURL();
      setImageData(imageUrl);
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

      <div>
        <h2>Camera Feed:</h2>
        {imageData ? (
          <img src={imageData} alt="Camera Feed" />
        ) : (
          <p>Waiting for image data...</p>
        )}
      </div>
    </div>
  );
};

export default ROSComponent;
