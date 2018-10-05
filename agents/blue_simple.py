import simple_agent

b = simple_agent.simple_agent('blue_sphero', opponent='red_sphero')

b.setup_ros()

b.play_game()
