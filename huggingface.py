# from transformers import GPT2Model, GPT2Tokenizer
# tokenizer = GPT2Tokenizer.from_pretrained('gpt2')
# model = GPT2Model.from_pretrained('gpt2')

from transformers import GPT2Tokenizer, GPT2LMHeadModel

import torch
# Load pre-trained model tokenizer (vocabulary)
tokenizer = GPT2Tokenizer.from_pretrained('gpt2')

# Encode the input text
input_text = "The quick brown fox jumps over the lazy dog"
input_tokens = tokenizer.encode(input_text, return_tensors='pt')

# Load pre-trained model (weights)
model = GPT2LMHeadModel.from_pretrained('gpt2')

# Set the model to evaluation mode to deactivate the Dropout modules
model.eval()

# Predict all tokens
with torch.no_grad():
    outputs = model(input_tokens)
    predictions = outputs[0]

# Get the predicted next token
predicted_index = torch.argmax(predictions[:, -1, :], dim=-1)
predicted_token = tokenizer.decode(predicted_index)

print(f'Input text: {input_text}')
print(f'Predicted next token: {predicted_token}')