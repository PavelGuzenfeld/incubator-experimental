import requests
import pandas as pd

# Fetch data from the URL
url = "https://blockchain.info/ticker"
response = requests.get(url)
data = response.json()

# Transform data into a Pandas DataFrame
df = pd.DataFrame(data).transpose()
df.reset_index(inplace=True)
df.columns = ["Currency", "15m", "Last", "Buy", "Sell", "Symbol"]

# Print the DataFrame in a table format
print(df.to_string(index=False))
