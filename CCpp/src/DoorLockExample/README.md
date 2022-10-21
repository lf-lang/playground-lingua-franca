### Sample requirements of Door lock system

* **Door lock control**
    - When the door lock button is pressed, the door is locked.
    - Unlocks the door when the door lock button is released.
    - When a door lock command is received from external system, the door is locked.
    - When a door unlock command is received from external system, the door is unlocked.
    - Door lock-unlock only works when the door is closed.
    - The time to lock the door shall be within 1 seconds after the button is pressed or the lock command is received from external system. The unlock time shall also be within 1 seconds as well.

* **Door open**
    - Door can be opened when the door is only unlocked.

* **Door open event notification**
    - Door open event is notified to external system when a door is opened.
    - Door close event is notified to external system when a door is closed.

* **Door locked event notification**
    - Door locked event is notified to external system when a door is locked.
    - Door unlocked event is notified to external system when a door is unlocked.  
