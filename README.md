## RL-with-CBF

### Install the dependancies 

```
pip install -r requirements.txt
```

### To run the PPO code (tested on Cartpole Environment - discrete action space) 

```
python ppo.py --track --capture-video 
```

Use Wanb library to visualise the loss functions and model performance

### To test on custom enviroments 

* Create an environment in OpenAI Gym format
* Wrap it using gym.make() function and test it for Gym compatibility 
* Vectorise the environment and run the PPO code 
