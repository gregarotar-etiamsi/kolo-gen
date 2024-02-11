import pandas as pd

# Read the CSV file into a DataFrame
df = pd.read_csv('mqtt.log')

# Insert a new column at the beginning with values incrementing from 1
df.insert(0, 'ID', range(1, len(df) + 1))

# Save the modified DataFrame back to a new CSV file
df.to_csv('modified_file.csv', index=False)
