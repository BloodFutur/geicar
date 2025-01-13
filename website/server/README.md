## To deploy website on server

- Install nodejs
- Install serve (node)
- Install pm2 (node)
- Install nginx

- Build: `npx vite build`
- Serve: `npx pm2 start serve --name "geicar" -- dist 3456`
- Save and set to startup: `npx pm2 save` `npx pm2 startup`
- Set up nginx server: use [nginx-geicar data](./config_files/nginx-geicar.txt)
- Add HTTPS with Let's Encrypt `sudo apt install certbot python3-certbot-nginx` `sudo certbot --nginx -d yourdomain.com`
- Enable the site `sudo ln -s /etc/nginx/sites-available/geicar /etc/nginx/sites-enabled/`
- Set up Mosquitto `sudo apt install mosquitto mosquitto-clients`  `sudo nano /etc/mosquitto/mosquitto.conf` (The conf is [here](./config_files/mosquitto.conf)])
- Do not forget to make a `.env` file with the following env variables:
    - REACT_APP_MQTT_BROKER_URL
    - REACT_APP_MQTT_CLIENT_ID
    - REACT_APP_MQTT_USERNAME
    - REACT_APP_MQTT_PASSWORD
- Test that it works, have fun