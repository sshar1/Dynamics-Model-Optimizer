from asammdf import MDF

# TODO this file is used to get the true state and write it to datasets
# as a csv.
# i.e. one thing we need to do is take gps data and convert it to
# x, y state.

mdf = MDF("car_logs/logfile2024-09-10_00-24-44.mdf")
df = mdf.to_dataframe()
print(df.head())

print(df.columns.tolist())
print(df["Movella_x"])
