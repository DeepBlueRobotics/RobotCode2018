package jaci.pathfinder;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class PathfinderTest {

	@Test
	/** Just a very basic test to ensure that the Pathfinder jar is installed, and we can reference it. **/
	void testPathfinder() {
		assertNotNull(new Pathfinder());
	}

}
