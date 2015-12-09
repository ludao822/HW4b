import Pilot
import time, os

# ___________ Uncomment these set of lines if in training mode. _____________

# # Delete previous training example if it exists.
# try:
#     os.remove('gamma.txt')
# except:
#     pass
#
# for i in range(-25, 25):
#     # Write current pitch test value to file
#     f = open('pitch.txt', 'w')
#     f.write(str(i))
#     f.close()
#
#     print('Testing for pitch angle of =', i, 'degrees')
#
#     a = Pilot.Pilot(tsk='HW4a', rc=True, gui=False)
#     a.start()
#
#     time.sleep(5)


# ___________ Uncomment these set of lines if in testing mode. _____________

a = Pilot.Pilot(tsk='HW4a', rc=True, gui=False, desired_altitude_change=2000, desired_gamma=10, desired_velocity=150)

# Check the pre-flight conditions to ensure that maneuver can be done
print("Checking the pre-flight conditions")
PC = a.PC()
print(PC, '\n')

# The PLAN function
print("Performing the plan function. ")
PLAN = a.PLAN()
print(PLAN, '\n')

# Please wait for 2 seconds before calling the DO function. This is to ensure that the under-lying operating system
# cleanly stops the UDP sockets
time.sleep(2)

# The DO function. The DO function here will just run the whole thing. If you want to call it repeatedly, change the DO
# function to call self.ai() instead of self.start()
if PC == 'OK':
    done = a.DO()
    print(done,'\n')
else:
    print("The PC conditions are not met. Not sure if you want to run it or not. Please change it accordingly. ")

