from watchdog_plus.managers import BaseObserverManager, StartMethods

items = [
    ["/Users/igor/Default.map", 'desktop'],
    ["/Users/igor/Default.map", 'downloads'],
]
# initialize the manager class
manager = BaseObserverManager()
# configure the handler class
# it takes extra args for log_file and filter_modified

manager.handler_config(filter_modified=True) # handler without log file, logs to stdout
# manager.handler_config(filter_modified=True, log_file="mylogfile.log") # handler with log_file

observers = manager.create_observers(items) # create observers from items

# start a single observer
manager.start_observer(name="desktop")
manager.start_observer(name="downloads")

# or start all the observers
manager.start_observers(observers=observers)