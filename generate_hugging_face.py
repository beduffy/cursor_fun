from transformers import GPT2Tokenizer, GPT2LMHeadModel

# Load pre-trained model tokenizer (vocabulary)
tokenizer = GPT2Tokenizer.from_pretrained('gpt2')

# Encode the input text
input_text = "The quick brown fox jumps over the lazy dog"
input_tokens = tokenizer.encode(input_text, return_tensors='pt')

# Load pre-trained model (weights)
model = GPT2LMHeadModel.from_pretrained('gpt2')

# Generate text
# max_length specifies the total length of the output text (input + generated tokens)
# num_return_sequences specifies how many different sequences to generate
output_sequences = model.generate(
    input_ids=input_tokens,
    max_length=50,  # Adjust this to generate more or fewer tokens
    num_return_sequences=1,  # Number of sentences to generate
    no_repeat_ngram_size=2,  # Ensures diversity by preventing repeating n-grams
    temperature=0.7,  # Controls randomness. Lower is less random.
    top_k=50,  # Keeps only top k tokens for sampling
    top_p=0.95,  # Nucleus sampling: keeps the top p cumulative probability
)

# Decode the generated text
generated_text = tokenizer.decode(output_sequences[0], skip_special_tokens=True)

print(f'Generated text: {generated_text}')