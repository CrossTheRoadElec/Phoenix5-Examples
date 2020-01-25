# Orchestra Example

This example demonstrates how to use the new Orchestra class in Phoenix to create music from your robot!

The basic method of operation is as follows:
 1. Create a Chirp(.chrp) file from a Midi file using Phoenix Tuner's Music Chirp Generator tab.
 2. Move the Chirp file onto the RoboRIO. In This example, this is done by copying the Chirp file into the src/main/deploy directory. This will deploy all the Chirp files onto the RIO at the /home/lvuser/deploy directory.
 3. Create an Orchestra object using Phoenix API.
 4. Load the Chirp file onto the Orchestra object using the LoadMusic routine. In this example, the chirp files are loaded by name and extension only, because Phoenix will assume they are at the /home/lvuser/deploy directory location. 
 5. Add TalonFX's to the Orchestra object. Each FX is a unique instrument.
 6. Call the play/pause/stop routines


#### Some things to note with this API.
 - The robot must be enabled to play music
 - Calling the set routine of any instrument will cause the Orchestra to pause.
