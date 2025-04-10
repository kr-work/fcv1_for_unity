import java.util.Arrays;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Structure;
import com.sun.jna.Pointer;
import com.sun.jna.Function;


public interface SimulatorLibrary extends Library {
  Map<String, Object> OPTIONS = new HashMap<String, Object>() {{
      put(Library.OPTION_CALLING_CONVENTION, Function.C_CONVENTION);
  }};
  SimulatorLibrary INSTANCE = Native.load("simulator", SimulatorLibrary.class, OPTIONS);

  Pointer create_plugin(Pointer stoneData);
  void destroy_plugin(Pointer plugin);
  void reset_stones(Pointer plugin);
  void set_stones(Pointer plugin);
  void set_velocity(Pointer plugin, float velocity_x, float velocity_y, float angular_velocity, int total_shot, int shot_per_team, int team_id);
  void set_status(Pointer plugin, int status);
  void check_rule(Pointer plugin);
  boolean step(Pointer plugin, int index, float coefficient);
}

// Declare the structures used in the C++ library
public static class Vector2 extends Structure {
    public static class ByValue extends Vector2 implements Structure.ByValue {}
    
    public float x;
    public float y;

    public Vector2() {
        x = 0.0f;
        y = 0.0f;
    }
    
    @Override
    protected List<String> getFieldOrder() {
        return Arrays.asList("x", "y");
    }
}

// Declare the structures to use store stones data
public static class StoneData extends Structure {
    public static class ByValue extends StoneData implements Structure.ByValue {}
    
    public Vector2 position;

    public StoneData() {
        position = new Vector2();
    }
    
    @Override
    protected List<String> getFieldOrder() {
        return Arrays.asList("position");
    }
}

void setup() {
  // Set the library path to the directory containing the shared library
  // "simulator.dll" file is located in the same directory as this sketch
  System.setProperty("jna.library.path", sketchPath("."));

  StoneData stoneData = new StoneData();
  StoneData[] stoneDataArray = (StoneData[]) stoneData.toArray(16);

  // Instantiate the simulator library
  SimulatorLibrary simulator = SimulatorLibrary.INSTANCE;
  // Create a plugin using the pointer of the first StoneData element
  Pointer plugin = simulator.create_plugin(stoneDataArray[0].getPointer());

  // Set the initial position of the stones
  for (int i = 0; i < stoneDataArray.length; i++) {
      stoneDataArray[i].write();
  }

  // Set the rule(0: five rock, 1: no-tick)
  simulator.set_status(plugin, 0);

  // Set the velocity of the stone which is thrown by the player
  simulator.set_velocity(plugin, 0.12f, 2.3f, 1.57f, 0, 0, 0);

  boolean flag = true;
  int count = 0;
  // Simulate the motion of the stones
  while (flag && count < 50000) {
    flag = simulator.step(plugin, -1, 1.0f);
    count++;
    for (int i = 0; i < stoneDataArray.length; i++) {
      stoneDataArray[i].read();
    }
  }

  // Print the position of the stones after the simulation
  for (int i = 0; i < stoneDataArray.length; i++) {
    println("Stone " + i + ": " + stoneDataArray[i].position.x + ", " + stoneDataArray[i].position.y);
  }

  // Set the velocity of the stone which is thrown by the opponent
  simulator.set_velocity(plugin, 0.08f, 3.6f, 1.57f, 1, 0, 1);
  flag = true;
  count = 0;

  // Simulate the motion of the stones again
  while (flag && count < 50000) {
    flag = simulator.step(plugin, -1, 1.0f);
    count++;
    for (int i = 0; i < stoneDataArray.length; i++) {
        stoneDataArray[i].read();
    }
  }

  for (int i = 0; i < stoneDataArray.length; i++) {
    println("Stone " + i + ": " + stoneDataArray[i].position.x + ", " + stoneDataArray[i].position.y);
  }
  simulator.check_rule(plugin);
  // Read the position of the stones after applying rule
  for (int i = 0; i < stoneDataArray.length; i++) {
      stoneDataArray[i].read();
  }
  
  // Print the position of the stones after applying rule
  for (int i = 0; i < stoneDataArray.length; i++) {
    println("Stone " + i + ": " + stoneDataArray[i].position.x + ", " + stoneDataArray[i].position.y);
  }
  // All the stones are reset to the initial position
  simulator.reset_stones(plugin);
  
  // Read the position of the stones after reset
  for (int i = 0; i < stoneDataArray.length; i++) {
    stoneDataArray[i].read();
  }
  // Print the position of the stones after reset
  for (int i = 0; i < stoneDataArray.length; i++) {
    println("Stone " + i + ": " + stoneDataArray[i].position.x + ", " + stoneDataArray[i].position.y);
  }
  // Destroy the plugin to free up resources
  simulator.destroy_plugin(plugin);
}
