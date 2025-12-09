## Project Name
Web Page Text Ingestion to Qdrant

## Purpose
Automate the extraction of text content from a sitemap of web pages, generate embeddings using Cohere’s API, and store these embeddings in a Qdrant vector database for semantic search.

## Inputs
- `sitemap_url`: URL of the sitemap containing the pages to ingest.
- Environment variables:
  - `QDRANT_URL` – Qdrant cloud URL
  - `QDRANT_API_KEY` – API key for Qdrant
  - `COHERE_API_KEY` – API key for Cohere embeddings
  - `QDRANT_COLLECTION_NAME` – Name of the collection in Qdrant

## Outputs
- Vector embeddings stored in the Qdrant collection.
- Logs printed to console with status of ingestion, URLs processed, and chunks saved.

## Functional Requirements
1. Fetch all URLs listed in the sitemap.
2. Extract text content from each URL using `trafilatura`.
3. Split text into chunks ≤4000 characters to fit embedding model constraints.
4. Generate embeddings for each chunk using Cohere `embed-english-v3.0` model.
5. Create a Qdrant collection if it doesn’t exist.
6. Insert chunks with embeddings, URL, and metadata into the Qdrant collection.
7. Run as a one-time ingestion pipeline.

## Non-Functional Requirements
- Handle pages with missing or empty text gracefully.
- Chunking should avoid cutting mid-sentence.
- Logging should provide visibility on progress and warnings.
- Securely use API keys from environment variables.

## Constraints
- Cohere embeddings are limited to 1024 dimensions.
- Each chunk ≤ 4000 characters.
- Only run once per dataset ingestion.
