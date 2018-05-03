// Define possible states and actions the Sphero may encounter
var distance_choices = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];
var speed_choices = [40, 50, 60, 70, 80, 90, 100, 110, 120, 130];
　
async function startProgram() {
    // Initialize table for state -> action -> reward
    var Q_table = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    // Zero out all possible table entries
    for (var index = 0; index < 10; index++) {
        Q_table[index] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    }
　
    resetAim();
    var previous_x = 0;
    var previous_y = 0;
　
    // Exploration period
    var exploration_cycles = 15 * 5; // ~4 seconds per cycle -> 5 minutes training
    for (var count_1 = 0; count_1 < exploration_cycles; count_1++) {
        // Pick a random goal and random action
        var distance_index = Math.floor(Math.random() * 10);
        var goal = distance_choices[distance_index];
　
        var speed_index = Math.floor(Math.random() * 10);
        var action = speed_choices[speed_index];
　
        // Perform action
        await roll(0.0, action, 1.0);
        await delay(1.0);
　
        // Calculate distance 
        var current_x = getLocation().x;
        var current_y = getLocation().y;
　
        // Calculate distance travelled
        var delta_x = current_x - previous_x;
        var delta_y = current_y - previous_y;
        var travel = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
　
        // Shape a reward for learning process
        var error = Math.abs(goal - travel);
        var reward = 1 - error / 100.;
　
        // Store reward for state:action in Q-table     
        Q_table[distance_index][speed_index] = (Q_table[distance_index][speed_index] + reward) / 2;
　
        // Indicate proximity to target with LEDs
        if (error > 0.5) {
            setMainLed({
                r: Math.floor(255 * error),
                g: 0,
                b: 0
            });
        } else {
            setMainLed({
                r: 0,
                g: Math.floor(255 * (1 - error)),
                b: 0
            });
        }
　
        // Roll back as close as possible to starting point, reset previous(x, y)
        await roll(0.0, -action, 1.0);
        await delay(1.0);
        previous_x = getLocation().x;
        previous_y = getLocation().y;
    }
    // Set LED Purple to indicate the program is switching modes
    setMainLed({
        r: 255,
        g: 0,
        b: 255
    });
    await delay(1.0);
　
    // Exploitation period - Demonstrate rolling to each distance target
    for (var distance_index = 0; distance_index < 10; distance_index++) {
        // Find action in Q-table with highest score for this distance value
        var highest_score = -1000;
        var selected_action = 0;
        for (var speed_index = 0; speed_index < 10; speed_index++) {
            if (Q_table[distance_index][speed_index] > highest_score) {
                highest_score = Q_table[distance_index][speed_index];
                selected_action = speed_choices[speed_index];
            }
        }
        // Execute action selected
        await roll(0.0, selected_action, 1.0);
        await delay(1.0);
　
        // Calculate distance 
        var current_x = getLocation().x;
        var current_y = getLocation().y;
　
        // Calculate distance travelled
        var delta_x = current_x - previous_x;
        var delta_y = current_y - previous_y;
        var travel = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
　
        // Calculate error
        var error = Math.abs(distance_choices[distance_index] - travel);
　
        // Indicate proximity to target with LEDs
        if (error > 0.5) {
            setMainLed({
                r: Math.floor(255 * error),
                g: 0,
                b: 0
            });
        } else {
            setMainLed({
                r: 0,
                g: Math.floor(255 * (1 - error)),
                b: 0
            });
        }
　
        // Return to origin
        await roll(0.0, -selected_action, 1.0);
        await delay(1.0);
        previous_x = getLocation().x;
        previous_y = getLocation().y;
    }
    // Set LED Blue to indicate the program is done running
    setMainLed({
        r: 0,
        g: 0,
        b: 255
    });
}
