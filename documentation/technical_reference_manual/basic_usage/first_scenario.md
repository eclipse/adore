### Running Your First Scenario
So you have emerged into the world as a new born by installing ADORe while
simultaneously downloading the entire internet. You look around and ahead you 
spot the ADORe CLI car on the horizon...


If you have been greeted with the ADORe CLI car then you are ready to run your
first scenario!
```
Welcome to the ADORe Development CLI Ubuntu 20.04.6 LTS (GNU/Linux 5.19.0-45-generic x86_64)

            ____ 
         __/  |_\__
        |           -. 
  ......'-(_)---(_)--' 
```

___
> **ℹ️ INFO:**
> If you have not seen the ADORe CLI car yet then please review the [Getting Started 🔗](../setup/getting_started.md) guide
___


#### Launching the ADORe CLI
If you have seen ADORe CLI car and you have an interactive shell prompt such as the
following:
```bash
ADORe CLI: adore git:(master)  (0)>  
```
otherwise start the cli with:
```bash
make cli
```

#### Running A Scenario
Once in the [ADORe CLI docker context 🔗](../system_and_development/adore_cli.md#how-do-i-know-if-i-am-in-the-adore-cli-context) you can run a scenario with `roslaunch`:
```bash
cd adore_scenarios
roslaunch baseline_test.launch
```

#### I have errors now what?

1. You can review the [Problems and Solutions 🔗](../problems_and_solutions.md) guide.
2. You can [Get Help 🔗](../getting_help.md). Send us an SOS and
   we will come to the rescue.

