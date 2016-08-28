Installation
============

The installation will bring you up and running in less that 30 minutes with an instance of
Skysense Planner connected to a simulated drone.

.. NOTE:: The installation includes ROS, Arducopter and Gazebo.
   This is probably the fastest way on Earth to install all these complementary packages in an automated fashion.

#. Create a Virtual Machine *vm1* with a clean installation of Ubuntu 14.04.4 LTS (Trusty).

   .. HINT:: For AWS users: You can use an AWS EC2 instance *Ubuntu Server 14.04 LTS (HVM), SSD Volume Type*. Do not use t2.micro free tiers, it's known to not work. Use at least t2.medium instances.

#. Make sure that TCP ports 22, 80 and 9090 on *vm1* are accessible from the network.

#. Add ``ubuntu ALL=(ALL) NOPASSWD: ALL`` at the end of the sudoers file on *vm1* with ``visudo``.

   .. HINT:: For AWS users: You can skip this step.

#. Setup a pair of SSH keys for the *ubuntu* user to access *vm1*. Need help?
   `How to setup the SSH Keys <https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2>`_.

   .. HINT:: For AWS users: The pair SSH keys is generated for you. Your private key is named *something.pem*.

#. **The next steps should be executed on your development machine (i.e., not vm1)**.
   Clone the Skysense Planner repository and enter the main directory:

   .. code-block:: bash

      git clone https://github.com/skyscode/planner.git
      cd planner

#. Create a pair of files *vm1.host* and *vm1_key* in the *planner/credentials* directory, where *vm1.host*
   contains the IP address and SSH port (format: 'ip:port') of *vm1* and vm1_key its SSH private key.
   By default, the SSH port is 22.

   .. HINT:: For AWS users: Rename the *something.pem* private key file to *vm1_key* **without extension**.

   Now You should be able to login on *vm1* as follows:

   .. code-block:: bash

      ssh -i credentials/vm1_key ubuntu@$(cat credentials/vm1.host)

   You can also use our utility scripts to access *vm1*:

   .. code-block:: bash

      utils/ssh.sh vm1


#. Install all required packages, Skysense Planner, Arducopter and Gazebo:

   .. code-block:: bash

      utils/setup.sh vm1

   Sit and relax, this step will take 20-30 minutes.

   .. HINT:: For AWS users: This step takes approximately 20 minutes with the suggested setup.

#. Restart the system:

   .. code-block:: bash

      utils/ssh.sh vm1 sudo reboot

Congratulations! Now You're ready to fly.
