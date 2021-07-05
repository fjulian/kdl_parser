uint16 cmd
uint16 Demonstration                       = 0  # Save current sim state as training sample
uint16 Features                            = 1  # Extract features
uint16 Rules                               = 2  # Build rules from current data
uint16 Classify                            = 3  # Classify current sim state
uint16 Inquire                             = 4  # Ask for a label for a given configuration
uint16 InquireResult                       = 5  # Reply to the inquiry
