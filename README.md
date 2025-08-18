# Crystal Chamber
A DIY chamber designed for crystal growing through the evaporation method with temperature and relative humidity control.

The chamber is built with easily accessible components, utilizes silica gel as a desiccant agent, a Peltier element for cooling, and an Arduino for monitoring and regulating the environmental conditions. It includes a display for exposition control with four operational modes (temperature and relative humidity, temperature only, humidity only, and no control). Additionally, a weight scale circuite can be incorporated to measure the mass variation of the solutions over time.

> Este projeto também está disponível em português, consulte a pasta ["Portugues"](https://github.com/MrVoelho/CrystalChamber/tree/main/Portugues)

![Stripe](https://github.com/user-attachments/assets/6d9b3c81-93b5-4a7c-9f26-396169a9b430)
## Building
A list of required components and the schematics to build your own Crystal Chamber are described in the Documentation File, together with the description of the control algorithm coded in the Arduino sketch (also fully commented) and some images for guidance.

The project was designed in stages, from monitoring to humidity control and finally temperature control. So in case you don't have access to the full list of materials, or just want to start slowly, no problem. Check out the `Connections` section of the Documentation and adjust the sketch accordingly to what is available.

Basic knowledge in electronics, including soldering and handling components, is required to build this project. 
## Operating
Since crystal growing is a time-consuming process, the algorithm was designed with autonomy in mind, to minimize the need for intervention. There are also some safety checks to ensure crystal quality (e.g. minimal humidity level) and operational conditions (e.g. in case of sensor failure). A description of the operational modes can also be found in the Documentation. 

Once the chamber is set up, set the temperature and relative humidity exposition ranges on the Sketch, upload it to your arduino, load some dry silica gel in the desiccant container and go grow some crystals.

## Some theory
For more information about crystal growing and the physics behind it, check the [Home-based science of crystal growing](https://youtu.be/u3r0Pdgs1Jw) video

For more content on crystal growing hobby, check the [Reddit r/crystalgrowing](https://www.reddit.com/r/crystalgrowing/).

## ChangeLog
v1.2.J (july-august/2025)
- Adoption of a continuous rotation servo for temperature control
- Updates in the algorithm

v1.1.G (december/2024) 
- First public version.

## License 
GPL v3.0 License
