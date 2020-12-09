Repo for the CS539 course project on Grasp Generation for Robots based on the Graspnet implementation.

For the tranfer training, first add following two arguments to the existing command that you use (do not include the `continue_train` argument)
```bash
 --niter 1000 --niter_decay 10000
```

Now in the newly generated `vae_lr...` folder, just copy all the files from `vae_pretrained` folder and start training with the following three arguments
```bash
 --niter 1000 --niter_decay 10000 --continue_train true 
```

(Ideally, if you use the argument for name of the folder to use for training, you don't need the first step, but I didn't test it, so not putting in the readme file)
=======

For installation, please follow the steps at the parent repository https://github.com/jsll/pytorch_6dof-graspnet.

Our implementation is explained on the project website: 
