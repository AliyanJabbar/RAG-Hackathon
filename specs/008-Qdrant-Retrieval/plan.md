## Step 1 – Setup
- Load environment variables using `python-dotenv`.
- Initialize Cohere client.
- Initialize Qdrant client.

## Step 2 – Embedding Generation
- Implement `get_embedding(text)` function.
- Use Cohere `embed-english-v3.0` model.
- Ensure output embedding matches Qdrant vector size.

## Step 3 – Data Retrieval
- Implement `retrieve_data(query)` function.
- Generate embedding for the query.
- Query Qdrant collection using the embedding.
- Retrieve top 5 points by similarity.
- Extract `text` from each point’s payload.

## Step 4 – Agent Tool Integration
- Decorate `retrieve_data` with `@function_tool`.
- Make function callable by agents for retrieval tasks.

## Step 5 – Testing & Validation
- Test `retrieve_data` with sample queries.
- Verify returned text matches expected semantic results.
