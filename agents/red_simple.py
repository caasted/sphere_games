import simple_agent

b = simple_agent.simple_agent('red_sphero', opponent='blue_sphero')

b.setup_ros()

b.play_game()
