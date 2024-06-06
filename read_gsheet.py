import pandas as pd
import gspread
from oauth2client.service_account import ServiceAccountCredentials

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

    # Convert the json to dataframe
    df = pd.DataFrame.from_dict(data)

    return df

if __name__ == "__main__":
    sheet_url = "https://docs.google.com/spreadsheets/d/15wgN0vMONW4tixOiHvTZHg0mQGBW5BfdaBg0EqjEwtM/edit#gid=100944524"
    sheet_name = "Tracking 2024"
    df = read_gsheet_to_df(sheet_url, sheet_name)
    print(df)

