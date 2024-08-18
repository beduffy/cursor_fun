import pandas as pd
import gspread
from oauth2client.service_account import ServiceAccountCredentials
from datetime import datetime

def read_gsheet_to_df(sheet_url, sheet_name):
    # Define the scope
    scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]

    # Add credentials to the account
    creds = ServiceAccountCredentials.from_json_keyfile_name('credentials.json', scope)

    # Authorize the clientsheet 
    client = gspread.authorize(creds)

    # Get the instance of the Spreadsheet
    sheet = client.open_by_url(sheet_url)

    # Get the first sheet of the Spreadsheet
    worksheet = sheet.worksheet(sheet_name)

    # Get all the records of the data
    # data = worksheet.get_all_records()  # does not work
    data = worksheet.get_values()

    # Use the first row as headers
    headers = data[0]
    rows = data[1:]

    # Convert the data to a DataFrame
    df = pd.DataFrame(rows, columns=headers)

    # Replace 'OK' with 0 in the DataFrame
    df.replace('OK', 0, inplace=True)

    # Convert numeric strings to floats
    df = df.applymap(lambda x: float(x) if x.replace('.', '', 1).isdigit() else x)

    df = df.applymap(lambda x: float(x.split()[0]) if isinstance(x, str) and x[0].isdigit() else x)

    return df


def calculate_weekly_scores(df, habit_columns):
    # Assume the first column is the date column
    date_column = df.columns[0]

    # Convert the date column to datetime
    df[date_column] = pd.to_datetime(df[date_column], format='%d/%m/%Y')

    # Set the date column as the index
    df.set_index(date_column, inplace=True)

    # Filter the DataFrame to only include dates up to the current week
    # current_week = datetime.now().isocalendar()[1]
    # df = df[df.index.isocalendar().week <= current_week]
    current_date = datetime.now()
    df = df[df.index <= current_date]

    # Resample the DataFrame by week (starting on Monday) and sum the habit columns
    weekly_scores = df[habit_columns].resample('W-MON').sum()

    return weekly_scores


def get_habit_columns(df):
    """
    Get columns with headers that begin with a number followed by a colon.
    """
    habit_columns = [col for col in df.columns if col and col[0].isdigit() and col[1] == ':']
    return habit_columns


def extract_habit_names(habit_columns):
    """
    Extract easier to read habit names by only extracting the name of the habit before "Num times".
    """
    habit_names = [col.split(' Num times')[0].strip() for col in habit_columns]
    return habit_names


def rename_habit_columns(df, habit_columns, habit_names):
    """
    Rename the habit columns in the DataFrame to the specific habit names.
    """
    rename_dict = dict(zip(habit_columns, habit_names))
    df.rename(columns=rename_dict, inplace=True)


def save_gsheet_to_file(df, file_path):
    """
    Save the DataFrame to a CSV file.

    Parameters:
    df (pd.DataFrame): The DataFrame to save.
    file_path (str): The path to the file where the DataFrame will be saved.
    """
    df.to_csv(file_path, index=False)
    print(f"DataFrame saved to {file_path}")


if __name__ == "__main__":
    sheet_url = "https://docs.google.com/spreadsheets/d/15wgN0vMONW4tixOiHvTZHg0mQGBW5BfdaBg0EqjEwtM/edit#gid=100944524"
    sheet_name = "Tracking 2024"
    pd.set_option('display.max_columns', None)
    # df = read_gsheet_to_df(sheet_url, sheet_name)
    # print(df)

    # for faster reading and testing
    df = pd.read_csv('life_track.csv')
    print(df)

    # for saving and reading faster and testing faster
    # save_gsheet_to_file(df, 'life_track.csv')

    print(df.columns)

    habit_columns = get_habit_columns(df)
    print('habit_columns:', habit_columns)

    habit_names = extract_habit_names(habit_columns)
    print('habit_names:', habit_names)

    rename_habit_columns(df, habit_columns, habit_names)
    print('Renamed columns:', df.columns)

    weekly_scores = calculate_weekly_scores(df, habit_names)
    print(weekly_scores)

