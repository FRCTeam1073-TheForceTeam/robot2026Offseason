package frc.robot.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashSet;
import java.util.Set;

import org.junit.jupiter.api.Test;

public class DashboardNamesTest {
  @Test
  public void testAllKeysAreUnique() {
    Set<String> keys = new HashSet<>();
    int duplicateCount = 0;

    for (DashboardNames value : DashboardNames.values()) {
      String key = value.getKey();
      if (!keys.add(key)) {
        System.err.println("Duplicate key found: " + key + " (" + value.name() + ")");
        duplicateCount++;
      }
    }

    assertEquals(0, duplicateCount, "Found " + duplicateCount + " duplicate keys in DashboardNames enum");
    assertEquals(DashboardNames.values().length, keys.size(),
        "Number of unique keys should match number of enum values");
  }
}
