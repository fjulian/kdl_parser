uint16 cmd
uint16 DEMONSTRATION                       = 0  # Save current sim state as training sample
uint16 FEATURES                            = 1  # Extract features
uint16 RULES                               = 2  # Build rules from current data
uint16 CLASSIFY                            = 3  # Classify current sim state
uint16 INQUIRE                             = 4  # Ask for a label for a given configuration
uint16 INQUIRE_RESULT                      = 5  # Reply to the inquiry
