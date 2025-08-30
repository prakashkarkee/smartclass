![Dashboard Screenshot](/images/project_images/universitylogo.png)


# Smart ClassRoom Project

      Prakash Karkee
      Brian Kiprop
      Lihini Karunarathne


# Smart Class Environment Dashboard

This project is a web-based dashboard for monitoring and visualizing real-time and historical sensor data from a smart classroom. It uses Firebase Realtime Database for data storage and Chart.js for data visualization.

![Dashboard Screenshot](/images/project_images/dashboard.jpg)
![Dashboard Screenshot](/images/project_images/chart_info.jpg)


## Features

- Real-time status display for sensors: AC, Heater, Humidity, Light, Motion, Noise, Temperature

![Dashboard Screenshot](/images/project_images/schematic_diagram.jpg)

- Historical data charts for each sensor
- Toggle between today's data and all historical data
- Responsive and modern UI

## Project Structure

```
Here is the flow chart depicting the logic diagram. 

![Dashboard Screenshot](/images/project_images/flowchart.jpg)

```

## Setup

1. **Clone the repository** and open the project folder.
2. **Update Firebase Configuration**  
   In `index.html`, replace the `apiKey` and `appId` in the `firebaseConfig` object with your own Firebase project credentials.
3. **Open `index.html` in your browser** to view the dashboard.

## Dependencies

- [Firebase JS SDK](https://firebase.google.com/docs/web/setup)
- [Chart.js](https://www.chartjs.org/)

## Notes

- Sensor images are stored in the `images/` folder.
- Authentication credentials are currently hardcoded for demonstration. For production, use secure authentication methods.

## License

This project is for educational purposes.