uint16 cmd
uint16 Demonstration                       = 0  # Save current sim state as positive training sample
uint16 BuildRules                          = 1  # Build rules from current data
uint16 Classify                            = 2  # Classify current sim state
uint16 Inquire                             = 3  # Ask for a label for a given configuration
uint16 InquireResult                       = 4  # Reply to the inquiry