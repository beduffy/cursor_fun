import requests
from simpletransformers.language_modeling import LanguageModelingModel


def fetch_text_data(url):
    """
    Fetches text data from a given URL.
    
    Args:
    - url (str): The URL to fetch the text data from.
    
    Returns:
    - str: The fetched text data.
    """
    response = requests.get(url)
    response.raise_for_status()  # Raises an HTTPError if the response status code is 4XX/5XX
    return response.text


def train_llm_on_text_data(text_data, model_type="distilgpt2", model_name="distilgpt2", output_dir="outputs/"):
    """
    Trains a Language Learning Model (LLM) on the provided text data.
    
    Args:
    - text_data (str): The text data to train the LLM on.
    - model_type (str, optional): The type of model to use. Defaults to "distilgpt2" for a smaller model size.
    - model_name (str, optional): The pretrained model name. Defaults to "distilgpt2" for a smaller model size.
    - output_dir (str, optional): The directory to save the trained model. Defaults to "outputs/".
    """
    # Save the text data to a file as required by Simple Transformers
    with open("train_data.txt", "w") as file:
        file.write(text_data)
    
    # Configure the model for a smaller memory footprint
    model_args = {
        "reprocess_input_data": True,
        "overwrite_output_dir": True,
        "num_train_epochs": 5,
        "train_batch_size": 8,  # Reduced batch size for lower memory usage
        "save_steps": -1,  # Disable model checkpoint saving
        "mlm": False  # Important for non-MLM models like DistilGPT-2
    }
    
    # Initialize the model with a smaller model type and name
    model = LanguageModelingModel(model_type, model_name, args=model_args, use_cuda=False)
    
    # Train the model
    model.train_model("train_data.txt", output_dir=output_dir)

    
if __name__ == "__main__":
    url = "https://www.gutenberg.org/files/1342/1342-0.txt"  # Pride and Prejudice by Jane Austen
    text_data = fetch_text_data(url)
    train_llm_on_text_data(text_data)
